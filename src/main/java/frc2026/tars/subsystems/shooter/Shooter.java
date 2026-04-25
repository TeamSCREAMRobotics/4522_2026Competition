package frc2026.tars.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANrange;
import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.math.Conversions;
import com.teamscreamrobotics.math.ScreamMath;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.GeomUtil;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc2026.tars.RobotState;
import frc2026.tars.controlboard.Controlboard;
import frc2026.tars.controlboard.Dashboard;
import frc2026.tars.subsystems.drivetrain.Drivetrain;
import frc2026.tars.subsystems.intake.IntakeRollers;
import frc2026.tars.subsystems.intake.IntakeRollers.IntakeRollersGoal;
import frc2026.tars.subsystems.intake.IntakeWrist;
import frc2026.tars.subsystems.intake.IntakeWrist.IntakeWristGoal;
import frc2026.tars.subsystems.leds.LED;
import frc2026.tars.subsystems.shooter.feeder.Feeder;
import frc2026.tars.subsystems.shooter.flywheel.Flywheel;
import frc2026.tars.subsystems.shooter.flywheel.FlywheelConstants;
import frc2026.tars.subsystems.shooter.hood.Hood;
import frc2026.tars.subsystems.shooter.rollers.Rollers;
import frc2026.tars.util.Util;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends SubsystemBase {
  private final Flywheel flywheel;
  private final Hood hood;
  private final Drivetrain drivetrain;
  private final RobotState robotState;
  private final IntakeWrist intakeWrist;
  private final IntakeRollers intakeRollers;
  private final Rollers rollers;
  private final Feeder feeder;
  private final LED led;
  // private final Hopper hopper;
  private Pose2d robotPose;
  private ChassisSpeeds robotSpeeds;
  private final String logPrefix = "Subsystems/Shooter/";

  public final CANrange beam = new CANrange(0);

  public final CANrange beamOne = new CANrange(1);

  // 0.75
  private final Debouncer beamDebouncer = new Debouncer(0.2, DebounceType.kFalling);

  @Getter @Setter public Translation2d target = new Translation2d();

  private boolean wantShoot = false;

  public enum ShooterState {
    NA,
    IDLE,
    STOWED,
    SHOOTING,
    FERRYING,
    INTAKE_UP
  }

  private enum IdleState {
    NA,
    IDLE_HUB,
    IDLE_FERRY_DEPOT,
    IDLE_FERRY_OUTPOST,
  }

  @Getter @Setter public ShooterState state = ShooterState.IDLE;
  @Getter @Setter private IdleState idleState = IdleState.NA;

  public Shooter(
      Flywheel flywheel,
      Hood hood,
      IntakeWrist intakeWrist,
      IntakeRollers intakeRollers,
      Feeder feeder,
      Rollers rollers,
      // Hopper hopper,
      LED led,
      Drivetrain drivetrain,
      RobotState robotState) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.intakeWrist = intakeWrist;
    this.intakeRollers = intakeRollers;
    this.rollers = rollers;
    this.feeder = feeder;
    // this.hopper = hopper;
    this.drivetrain = drivetrain;
    this.robotState = robotState;
    this.led = led;

    beam.getConfigurator().apply(ShooterConstants.beamConfig);
    beamOne.getConfigurator().apply(ShooterConstants.beamConfig);

    beamOne.getIsDetected().setUpdateFrequency(100.0);
  }

  public Length getShotDistance(Translation2d target) {
    return Length.fromMeters(getFieldToShooter().getTranslation().getDistance(target));
  }

  // Replace your old sotmTof/sotmPrevVx/sotmPrevVy fields with these:
  private static final double SOTM_PHASE_DELAY_SECS = 0.02; // tune this
  private static final double SOTM_LINEAR_DRAG_K = 0.275; // tune this — 6328's proven value

  private void shootOnTheFly(
      Pose2d robotPose, ChassisSpeeds robotSpeeds, Translation2d target, boolean wantShoot) {

    // Step 1: Phase-delayed launcher position to account for actuator latency.
    // Move the robot pose forward a tiny bit before computing the launcher position.
    Translation2d launcherPos = getFieldToShooter().getTranslation();
    Translation2d phasedLauncherPos =
        new Translation2d(
            launcherPos.getX() + robotSpeeds.vxMetersPerSecond * SOTM_PHASE_DELAY_SECS,
            launcherPos.getY() + robotSpeeds.vyMetersPerSecond * SOTM_PHASE_DELAY_SECS);

    // Step 2: Compute the field-relative velocity AT the launcher (not robot center).
    // Accounts for the tangential velocity added by robot rotation.
    double launcherOffX = launcherPos.getX() - robotPose.getX();
    double launcherOffY = launcherPos.getY() - robotPose.getY();
    double omega = robotSpeeds.omegaRadiansPerSecond;
    double vx = robotSpeeds.vxMetersPerSecond + (-launcherOffY) * omega;
    double vy = robotSpeeds.vyMetersPerSecond + (launcherOffX) * omega;

    // Step 3: Iteratively find a self-consistent lookahead position and distance.
    // The ball travels with robot velocity imparted to it, so we project forward
    // by effectiveTof (not raw tof) to account for air drag reducing that offset.
    Translation2d lookaheadPos = phasedLauncherPos;
    double lookaheadDist = phasedLauncherPos.getDistance(target);

    for (int i = 0; i < 20; i++) {
      double tof = getTimeOfFlightForDistance(lookaheadDist);

      // Linear drag model: horizontal velocity decays as v(t) = v0 * e^(-k*t)
      // Effective displacement = v0 * integral(0, tof) e^(-kt) dt = v0 * (1 - e^(-k*tof)) / k
      // This prevents overcorrection at speed vs. naive v * tof.
      double effectiveTof = (1.0 - Math.exp(-tof * SOTM_LINEAR_DRAG_K)) / SOTM_LINEAR_DRAG_K;

      lookaheadPos =
          new Translation2d(
              phasedLauncherPos.getX() + vx * effectiveTof,
              phasedLauncherPos.getY() + vy * effectiveTof);
      lookaheadDist = lookaheadPos.getDistance(target);
    }

    // Step 4: Aim the drivetrain from the lookahead position toward the target.
    // This is equivalent to aiming at the "virtual target" corrected for robot motion.
    robotState.setDrivetrainTarget(
        ScreamMath.calculateAngleToPoint(lookaheadPos, target).plus(Rotation2d.k180deg));

    // Step 5: Flywheel and hood use the lookahead distance, not the static distance.
    double multiplier = wantShoot ? 1.0 : 2.0;
    hood.moveToAngle(
        wantShoot
            ? Rotation2d.fromDegrees(getHoodAngleFromDistance(lookaheadDist))
            : Rotation2d.kZero);
    flywheel.setTargetVelocityTorqueCurrent(
        ShooterConstants.NEW_FLYWHEEL_MAP.get(lookaheadDist) / multiplier, 0.0);

    Logger.log("SOTM/LookaheadDist", lookaheadDist);
    Logger.log("SOTM/LookaheadPos", new Pose2d(lookaheadPos, robotPose.getRotation()));
    Logger.log("SOTM/Target", target);
    Logger.log("SOTM/LauncherVx", vx);
    Logger.log("SOTM/LauncherVy", vy);
  }

  /*
   * Inspired by 6328, Mechanical Advantage
   * https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java
   */
  // private void launchCalcShootOnTheFly(
  //     Pose2d robotPose, ChassisSpeeds robotSpeeds, Translation2d target, boolean wantShoot) {

  //   double phaseDelay = ShooterConstants.SOTF_PHASE_DELAY;
  //   Pose2d phasePose =
  //       new Pose2d(
  //           robotPose.getX() + robotSpeeds.vxMetersPerSecond * phaseDelay,
  //           robotPose.getY() + robotSpeeds.vyMetersPerSecond * phaseDelay,
  //           robotPose
  //               .getRotation()
  //               .plus(Rotation2d.fromRadians(robotSpeeds.omegaRadiansPerSecond * phaseDelay)));

  //   Pose2d phasedLauncherPose =
  //       GeomUtil.transformToPose(
  //           GeomUtil.poseToTransform(phasePose)
  //               .plus(Util.transform3dTo2dXY(ShooterConstants.flywheelToRobot)));
  //   Translation2d launcherPos = phasedLauncherPose.getTranslation();

  //   double offX = launcherPos.getX() - robotPose.getX();
  //   double offY = launcherPos.getY() - robotPose.getY();
  //   double omega = robotSpeeds.omegaRadiansPerSecond;
  //   double vx = robotSpeeds.vxMetersPerSecond - offY * omega;
  //   double vy = robotSpeeds.vyMetersPerSecond + offX * omega;

  //   double k = ShooterConstants.SOTF_LINEAR_DRAG_CONSTANT;
  //   double tof = getTimeOfFlightForDistance(launcherPos.getDistance(target));
  //   double projDist = launcherPos.getDistance(target);

  //   for (int i = 0; i < 20; i++) {
  //     double effectiveTof = (1.0 - Math.exp(-tof * k)) / k;
  //     double futX = launcherPos.getX() + vx * effectiveTof;
  //     double futY = launcherPos.getY() + vy * effectiveTof;
  //     projDist = Math.hypot(futX - target.getX(), futY - target.getY());
  //     double newTof = getTimeOfFlightForDistance(projDist);
  //     if (Math.abs(newTof - tof) < 0.001) {
  //       tof = newTof;
  //       break;
  //     }
  //     tof = newTof;
  //   }

  //   double effectiveTof = (1.0 - Math.exp(-tof * k)) / k;
  //   Translation2d futurePos =
  //       new Translation2d(
  //           launcherPos.getX() + vx * effectiveTof, launcherPos.getY() + vy * effectiveTof);
  //   double futureDistance = futurePos.getDistance(target);

  //   double hoodAngleRad = Units.degreesToRadians(getHoodAngleFromDistance(futureDistance));
  //   if (Double.isNaN(lcLastHoodAngleRad)) lcLastHoodAngleRad = hoodAngleRad;
  //   double hoodVelocity = lcHoodAngleFilter.calculate((hoodAngleRad - lcLastHoodAngleRad) /
  // 0.02);
  //   lcLastHoodAngleRad = hoodAngleRad;

  //   Rotation2d driveAngle =
  //       ScreamMath.calculateAngleToPoint(futurePos, target).plus(Rotation2d.k180deg);
  //   if (lcLastDriveAngle == null) lcLastDriveAngle = driveAngle;
  //   double driveVelocity =
  //       lcDriveAngleFilter.calculate(driveAngle.minus(lcLastDriveAngle).getRadians() / 0.02);
  //   lcLastDriveAngle = driveAngle;

  //   robotState.setDrivetrainTarget(driveAngle);

  //   double multiplier = wantShoot ? 1.0 : 2.0;
  //   hood.moveToAngle(wantShoot ? Rotation2d.fromRadians(hoodAngleRad) : Rotation2d.kZero);
  //   flywheel.setTargetVelocityTorqueCurrent(
  //       ShooterConstants.NEW_FLYWHEEL_MAP.get(futureDistance) / multiplier, 0.0);

  //   Logger.log("SOTM/ToF", tof);
  //   Logger.log("SOTM/FutureDistance", futureDistance);
  //   Logger.log("SOTM/FuturePose", new Pose2d(futurePos, robotPose.getRotation()));
  //   Logger.log("SOTM/Target", target);
  //   Logger.log("SOTM/LauncherVx", vx);
  //   Logger.log("SOTM/LauncherVy", vy);
  //   Logger.log("SOTM/HoodVelocity", hoodVelocity);
  //   Logger.log("SOTM/DriveVelocity", driveVelocity);
  // }

  private double getTimeOfFlightForDistance(double distanceMeters) {
    double flywheelVelocity = ShooterConstants.NEW_FLYWHEEL_MAP.get(distanceMeters);
    double hoodAngle = getHoodAngleFromDistance(distanceMeters);
    return getTimeOfFlight(distanceMeters, flywheelVelocity, hoodAngle);
  }

  private void applyAimingSetpoints(
      Pose2d robotPose, ChassisSpeeds robotSpeeds, Translation2d target, boolean wantShoot) {
    if (!Dashboard.dissableShootOnTheMove.get()) {
      shootOnTheFly(robotPose, robotSpeeds, target, wantShoot);
    } else {
      setTarget(target);
      double distanceMeters = getShotDistance(target).getMeters();
      double hoodAngleDeg = getHoodAngleFromDistance(distanceMeters);

      double multiplier = wantShoot ? 1.0 : 3.0;
      double flywheelMap = ShooterConstants.NEW_NEW_FLYWHEEL_MAP.get(distanceMeters);

      double flywheelSetpoint;
      if (distanceMeters <= 2.0) {
        flywheelSetpoint = flywheelMap * Dashboard.closeMapNudge.get();
      } else if (distanceMeters <= 4.0) {
        flywheelSetpoint = flywheelMap * Dashboard.midMapNudge.get();
      } else {
        flywheelSetpoint = flywheelMap * Dashboard.farMapNudge.get();
      }

      robotState.setDrivetrainTarget(
          ScreamMath.calculateAngleToPoint(robotPose.getTranslation(), target)
              .plus(Rotation2d.k180deg));

      hood.moveToAngle(Rotation2d.fromDegrees(wantShoot ? hoodAngleDeg : 0.0));
      flywheel.setTargetVelocityTorqueCurrent(flywheelSetpoint / multiplier, 0.0);
      // flywheel.setTargetVelocityTorqueCurrent(Dashboard.flywheelVelocity.get(), 0.0);

      Logger.log(logPrefix + "Hood Angle", hoodAngleDeg);
      Logger.log(logPrefix + "Flywheel Velocity", flywheelSetpoint);
      Logger.log(logPrefix + "Shot Distance", distanceMeters);
    }
  }

  public Command autoShoot() {
    return Commands.parallel(
            Commands.run(() -> wantShoot = true),
            intakeWrist.compress(() -> robotState.atTargetAngle()))
        .withDeadline(
            new WaitUntilCommand(() -> !beamDebouncer.calculate(beam.getIsDetected().getValue())))
        .finallyDo(() -> wantShoot = false);
  }

  public void runFeed(Translation2d target) {
    if ((flywheel.atVel() || Dashboard.disableWaitUntilAtVelocity.get())
        && (Math.abs(hood.getError()) <= 0.1 || Dashboard.disableWaitUntilHood.get())
        && robotState.atTargetAngle()) {
      rollers.setVoltage(12.0);
      feeder.setVoltage(12.0);
      // led.solid(Color.kRed);
    }
  }

  public void stopFeed() {
    rollers.setVoltage(0.0);
    feeder.setVoltage(0.0);
  }

  private double agitateStartTime = 0.0;
  private boolean agitateForward = true;

  public void agitate(boolean shouldAgitate) {
    if (!shouldAgitate) {
      intakeWrist.applyGoal(IntakeWristGoal.EXTENDED);
      intakeRollers.applyGoal(IntakeRollersGoal.STOP);
      return;
    }

    double now = Timer.getFPGATimestamp();

    if (agitateStartTime == 0.0) {
      agitateStartTime = now;
    }

    if (now - agitateStartTime >= 0.5) {
      agitateForward = !agitateForward;
      agitateStartTime = now;
    }

    if (agitateForward) {
      intakeRollers.applyGoal(IntakeRollersGoal.INTAKE);
      intakeWrist.applyGoal(IntakeWristGoal.AGITATE_HIGH);
    } else {
      intakeWrist.applyGoal(IntakeWristGoal.EXTENDED);
    }
  }

  private void idleCase(RobotState.Area area, Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    if (area == null) return;
    switch (area) {
      case ALLIANCEZONE:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(FieldConstants.Hub.hubCenter, FieldConstants.Hub.oppHubCenter),
            wantShoot);
        setIdleState(IdleState.IDLE_HUB);
        led.wave(
            Color.kBlack,
            AllianceFlipUtil.get(
                new Color(1.0f, 0.49803922f, 0.83137256f),
                new Color(0.26078432f, 1.0f, 0.36078432f)),
            0.1,
            1.25);
        break;
      case DEPOT_SIDE_NEUTRALZONE:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.leftAllianceZone,
                FieldConstants.AllianceZones.oppRightAllianceZone),
            wantShoot);
        setIdleState(IdleState.IDLE_FERRY_DEPOT);
        led.wave(Color.kBlack, new Color(0.0f, 0.5019608f, 0.5019608f), 0.1, 1.25);
        break;
      case OUTPOST_SIDE_NEUTRALZONE:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.rightAllianceZone,
                FieldConstants.AllianceZones.oppLeftAllianceZone),
            wantShoot);
        setIdleState(IdleState.IDLE_FERRY_OUTPOST);
        led.wave(Color.kBlack, new Color(0.0f, 0.5019608f, 0.5019608f), 0.1, 1.25);
        break;
      case OTHER_ALLIANCEZONE_DEPOT:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.rightAllianceZone,
                FieldConstants.AllianceZones.oppLeftAllianceZone),
            wantShoot);
        setIdleState(IdleState.IDLE_FERRY_DEPOT);
        led.wave(
            Color.kBlack,
            AllianceFlipUtil.get(
                new Color(0.26078432f, 1.0f, 0.36078432f) /*new Color(0.0f, 1.0f, 0.83137256f)*/,
                new Color(1.0f, 0.49803922f, 0.83137256f)),
            0.1,
            1.25);
        break;
      case OTHER_ALLIANCEZONE_OUTPOST:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.leftAllianceZone,
                FieldConstants.AllianceZones.oppRightAllianceZone),
            wantShoot);
        setIdleState(IdleState.IDLE_FERRY_OUTPOST);
        led.wave(
            Color.kBlack,
            AllianceFlipUtil.get(
                new Color(0.26078432f, 1.0f, 0.36078432f) /*new Color(0.0f, 1.0f, 0.83137256f)*/,
                new Color(1.0f, 0.49803922f, 0.83137256f)),
            0.1,
            1.25);
        break;

      default:
        setIdleState(IdleState.NA);
        break;
    }
  }

  private void ferryCase(
      RobotState.Area area, Pose2d robotPose, ChassisSpeeds robotSpeeds, boolean wantShoot) {
    if (area == null) return;

    switch (area) {
      case DEPOT_SIDE_NEUTRALZONE:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.leftAllianceZone,
                FieldConstants.AllianceZones.oppRightAllianceZone),
            wantShoot);
        runFeed(
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.leftAllianceZone,
                FieldConstants.AllianceZones.oppRightAllianceZone));
        break;
      case OUTPOST_SIDE_NEUTRALZONE:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.rightAllianceZone,
                FieldConstants.AllianceZones.oppLeftAllianceZone),
            wantShoot);
        runFeed(
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.rightAllianceZone,
                FieldConstants.AllianceZones.oppLeftAllianceZone));
        break;
      case OTHER_ALLIANCEZONE_DEPOT:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.rightAllianceZone,
                FieldConstants.AllianceZones.oppLeftAllianceZone),
            wantShoot);
        runFeed(
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.rightAllianceZone,
                FieldConstants.AllianceZones.oppLeftAllianceZone));
        break;
      case OTHER_ALLIANCEZONE_OUTPOST:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.leftAllianceZone,
                FieldConstants.AllianceZones.oppRightAllianceZone),
            wantShoot);
        runFeed(
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.leftAllianceZone,
                FieldConstants.AllianceZones.oppRightAllianceZone));
        break;
      default:
        stopFeed();
        break;
    }
  }

  private void updateShooterState(RobotState.Area area, boolean wantShoot) {
    if (Controlboard.moveIntakeWrist().getAsBoolean()) {
      setState(ShooterState.INTAKE_UP);
    } else if (wantShoot && area == RobotState.Area.ALLIANCEZONE) {
      setState(ShooterState.SHOOTING);
    } else if (wantShoot
        && (area == RobotState.Area.DEPOT_SIDE_NEUTRALZONE
            || area == RobotState.Area.OUTPOST_SIDE_NEUTRALZONE
            || area == RobotState.Area.OTHER_ALLIANCEZONE_DEPOT
            || area == RobotState.Area.OTHER_ALLIANCEZONE_OUTPOST)) {
      setState(ShooterState.FERRYING);
    } else if (Dashboard.manualMode.get()) {
      setState(ShooterState.NA);
    } else {
      setState(ShooterState.IDLE);
    }
  }

  public Command defaultCommand() {
    return run(() -> {
          RobotState.Area area = robotState.getArea();
          robotPose = drivetrain.getEstimatedPose();
          robotSpeeds = drivetrain.getFieldVelocity();
          if (!DriverStation.isAutonomous()) {
            wantShoot = Controlboard.shoot().getAsBoolean();
          }

          Logger.log(logPrefix + "Area", area != null ? area.toString() : "NULL");
          Logger.log(logPrefix + "RobotPose X", robotPose.getX());
          Logger.log(logPrefix + "RobotPose Y", robotPose.getY());
          Logger.log(logPrefix + "wantShoot", wantShoot);
          updateShooterState(area, wantShoot);

          switch (state) {
            case IDLE:
              idleCase(area, robotPose, robotSpeeds);
              agitateStartTime = 0.0;
              agitateForward = true;
              stopFeed();

              break;

            case SHOOTING:
              applyAimingSetpoints(
                  robotPose,
                  robotSpeeds,
                  AllianceFlipUtil.get(
                      FieldConstants.Hub.hubCenter, FieldConstants.Hub.oppHubCenter),
                  wantShoot);
              runFeed(
                  AllianceFlipUtil.get(
                      FieldConstants.Hub.hubCenter, FieldConstants.Hub.oppHubCenter));
              setIdleState(IdleState.NA);
              break;

            case FERRYING:
              ferryCase(area, robotPose, robotSpeeds, wantShoot);
              setIdleState(IdleState.NA);
              agitateStartTime = 0.0;
              agitateForward = false;
              break;

            case INTAKE_UP:
              hood.moveToAngle(Rotation2d.fromDegrees(0.0));
              agitateStartTime = 0.0;
              agitateForward = true;
              // flywheel.setTargetVelocityTorqueCurrent(ShooterConstants.FLYWHEEL_MAP.get(getShotDistance(AllianceFlipUtil.get(FieldConstants.Hub.hubCenter, FieldConstants.Hub.oppHubCenter)).getMeters()), 0.0);
              break;
            case NA:
              agitateStartTime = 0.0;
              agitateForward = true;
              break;

            default:
              agitateStartTime = 0.0;
              agitateForward = true;
              break;
          }
        })
        .withName("Shooter Default Command");
  }

  @Override
  public void periodic() {
    Logger.log("Shooter/Shooter State", getState().toString());
    Logger.log("Shooter/Idle State", getIdleState().toString());
    if (robotPose != null) {
      Logger.log(
          "Shooter/Field To Turret",
          new Pose3d(
              getFieldToShooter().getX(), getFieldToShooter().getY(), 0.5, Rotation3d.kZero));
    }

    Logger.log(logPrefix + "beam", beamDebouncer.calculate(beam.getIsDetected().getValue()));
  }

  public Pose2d getFieldToShooter() {
    Pose2d pose = robotPose;

    return GeomUtil.transformToPose(
        GeomUtil.poseToTransform(pose)
            .plus(Util.transform3dTo2dXY(ShooterConstants.flywheelToRobot)));
  }

  public double getTimeOfFlight(double distance, double velocity, double hoodAngleDeg) {
    double exitVelocity =
        Conversions.rpsToMPS(
                velocity,
                FlywheelConstants.FLYWHEEL_CIRCUMFERENCE.getMeters(),
                FlywheelConstants.FLYWHEEL_REDUCTION)
            * 0.8;
    double exitAngle = 90.0 - hoodAngleDeg;
    double horizontalVelocity = exitVelocity * Math.cos(Units.degreesToRadians(exitAngle));

    if (horizontalVelocity < 0.01) return distance / 0.01;
    return distance / horizontalVelocity;
  }

  public double getHoodAngleFromDistance(double distance) {
    // return Dashboard.saturationLevel.get() * (1 - Math.pow(Math.E, -(Dashboard.functionROA.get()
    // * distance))) + (Dashboard.functionLRG.get() * Math.pow(distance, 3));
    if (distance < 2.0) {
      return 0.0;
    } else if (distance > 6.3) {
      return 50.0;
    } else {
      return distance * Dashboard.functionScalar.get();
    }
  }
}
