package frc2026.tars.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANrange;
import com.team5000.Util;
import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.gameutil.FieldConstants;
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
import frc2026.tars.subsystems.shooter.hood.Hood;
import frc2026.tars.subsystems.shooter.rollers.Rollers;
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
  private Pose2d robotPose;
  private ChassisSpeeds robotSpeeds;
  private RobotState.Area currentArea = RobotState.Area.UNKNOWN;
  private final String logPrefix = "Subsystems/Shooter/";

  public final CANrange beam = new CANrange(0);

  public final CANrange beamOne = new CANrange(1);

  private Debouncer autoShootBeamDebouncer = new Debouncer(0.2, DebounceType.kFalling);

  @Getter @Setter public Translation2d target = new Translation2d();

  private boolean wantShoot = false;
  private boolean feedLatched = false;

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

    beam.getIsDetected().setUpdateFrequency(100.0);
    beamOne.getIsDetected().setUpdateFrequency(100.0);
  }

  public Length getShotDistance(Translation2d target) {
    return getShotDistance(drivetrain.getEstimatedPose(), target);
  }

  private Length getShotDistance(Pose2d robotPose, Translation2d target) {
    return Length.fromMeters(getFieldToShooter(robotPose).getTranslation().getDistance(target));
  }

  private void applyAimingSetpoints(
      Pose2d robotPose, ChassisSpeeds robotSpeeds, Translation2d target, boolean wantShoot) {
    setTarget(target);
    double distanceMeters = getShotDistance(robotPose, target).getMeters();
    Rotation2d drivetrainTarget =
        ScreamMath.calculateAngleToPoint(robotPose.getTranslation(), target)
            .plus(Rotation2d.k180deg);

    if (!Dashboard.disableShootOnTheMove.get()) {
      ShootOnTheMoveCalculator.Solution solution =
          ShootOnTheMoveCalculator.calculate(
              robotPose,
              getFieldToShooter(robotPose),
              robotSpeeds != null ? robotSpeeds : new ChassisSpeeds(),
              target,
              this::getFlywheelSetpoint,
              this::getHoodAngleFromDistance,
              Dashboard.shootOnTheMovePhaseDelay.get(),
              Dashboard.shootOnTheMoveLinearDrag.get());

      distanceMeters = solution.correctedDistanceMeters();
      drivetrainTarget = solution.drivetrainHeading();

      Logger.log(logPrefix + "SOTM/Enabled", true);
      Logger.log(logPrefix + "SOTM/CorrectedLaunchPosition", solution.correctedLaunchPosition());
      Logger.log(logPrefix + "SOTM/LauncherVx", solution.launcherFieldVelocity().vxMetersPerSecond);
      Logger.log(logPrefix + "SOTM/LauncherVy", solution.launcherFieldVelocity().vyMetersPerSecond);
      Logger.log(logPrefix + "SOTM/TimeOfFlight", solution.timeOfFlightSeconds());
    } else {
      Logger.log(logPrefix + "SOTM/Enabled", false);
    }

    double hoodAngleDeg = getHoodAngleFromDistance(distanceMeters);

    double multiplier = wantShoot ? 1.0 : 3.0;
    double flywheelSetpoint = getFlywheelSetpoint(distanceMeters);

    robotState.setDrivetrainTarget(drivetrainTarget);

    hood.moveToAngle(Rotation2d.fromDegrees(wantShoot ? hoodAngleDeg : 0.0));
    flywheel.setTargetVelocityTorqueCurrent(flywheelSetpoint / multiplier, 0.0);

    Logger.log(logPrefix + "Hood Angle", hoodAngleDeg);
    Logger.log(logPrefix + "Flywheel Velocity", flywheelSetpoint);
    Logger.log(logPrefix + "Shot Distance", distanceMeters);
    Logger.log(logPrefix + "Point", target);
  }

  private double getFlywheelSetpoint(double distanceMeters) {
    double flywheelMap = ShooterConstants.NEW_NEW_FLYWHEEL_MAP.get(distanceMeters);

    if (distanceMeters <= 2.0) {
      return flywheelMap * Dashboard.closeMapNudge.get();
    } else if (distanceMeters <= 4.0) {
      return flywheelMap * Dashboard.midMapNudge.get();
    } else {
      return flywheelMap * Dashboard.farMapNudge.get();
    }
  }

  public Command autoShoot() {
    return Commands.parallel(
            Commands.run(() -> wantShoot = true),
            intakeWrist.compress(() -> robotState.atTargetAngle()))
        .withDeadline(
            new WaitUntilCommand(
                () -> !autoShootBeamDebouncer.calculate(beam.getIsDetected().getValue())))
        .beforeStarting(() -> autoShootBeamDebouncer = new Debouncer(0.2, DebounceType.kFalling))
        .finallyDo(() -> wantShoot = false);
  }

  public void runFeed() {
    feedLatched = feedLatched || readyToFeed();

    if (feedLatched) {
      rollers.setVoltage(12.0);
      feeder.setVoltage(12.0);
    } else {
      rollers.setVoltage(0.0);
      feeder.setVoltage(0.0);
    }
  }

  private boolean readyToFeed() {
    return (flywheel.atVel() || Dashboard.disableWaitUntilAtVelocity.get())
        && (Math.abs(hood.getError()) <= 0.1 || Dashboard.disableWaitUntilHood.get())
        && robotState.atTargetAngle();
  }

  public void stopFeed() {
    feedLatched = false;
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
      intakeRollers.applyGoal(IntakeRollersGoal.AGITATE);
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
        break;
      case OTHER_ALLIANCEZONE_DEPOT:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(FieldConstants.rightMiddle, FieldConstants.leftMiddle),
            wantShoot);
        setIdleState(IdleState.IDLE_FERRY_DEPOT);
        break;
      case OTHER_ALLIANCEZONE_OUTPOST:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(FieldConstants.leftMiddle, FieldConstants.rightMiddle),
            wantShoot);
        setIdleState(IdleState.IDLE_FERRY_OUTPOST);
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
        runFeed();
        break;
      case OUTPOST_SIDE_NEUTRALZONE:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.rightAllianceZone,
                FieldConstants.AllianceZones.oppLeftAllianceZone),
            wantShoot);
        runFeed();
        break;
      case OTHER_ALLIANCEZONE_DEPOT:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(FieldConstants.rightMiddle, FieldConstants.leftMiddle),
            wantShoot);
        runFeed();
        break;
      case OTHER_ALLIANCEZONE_OUTPOST:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(FieldConstants.leftMiddle, FieldConstants.rightMiddle),
            wantShoot);
        runFeed();
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
    return Commands.run(
            () -> {
              RobotState.Area area = robotState.getArea();
              currentArea = area;
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
                  runFeed();
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
            },
            this,
            flywheel,
            hood,
            rollers,
            feeder)
        .withName("Shooter Default Command");
  }

  public void applyLedState() {
    if (state != ShooterState.IDLE) {
      robotState.flashLEDS();
      return;
    }

    switch (currentArea) {
      case ALLIANCEZONE:
        led.wave(
            Color.kBlack,
            AllianceFlipUtil.get(
                new Color(1.0f, 0.49803922f, 0.83137256f),
                new Color(0.26078432f, 1.0f, 0.36078432f)),
            0.1,
            1.25);
        break;
      case DEPOT_SIDE_NEUTRALZONE:
      case OUTPOST_SIDE_NEUTRALZONE:
        led.wave(Color.kBlack, new Color(0.0f, 0.5019608f, 0.5019608f), 0.1, 1.25);
        break;
      case OTHER_ALLIANCEZONE_DEPOT:
      case OTHER_ALLIANCEZONE_OUTPOST:
        led.wave(
            Color.kBlack,
            AllianceFlipUtil.get(
                new Color(0.26078432f, 1.0f, 0.36078432f),
                new Color(1.0f, 0.49803922f, 0.83137256f)),
            0.1,
            1.25);
        break;
      default:
        robotState.flashLEDS();
        break;
    }
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

    Logger.log(logPrefix + "beam", beam.getIsDetected().getValue());
  }

  public Pose2d getFieldToShooter() {
    return getFieldToShooter(robotPose != null ? robotPose : drivetrain.getEstimatedPose());
  }

  private Pose2d getFieldToShooter(Pose2d pose) {
    return GeomUtil.transformToPose(
        GeomUtil.poseToTransform(pose)
            .plus(Util.transform3dTo2dXY(ShooterConstants.flywheelToRobot)));
  }

  public double getHoodAngleFromDistance(double distance) {
    // return Dashboard.saturationLevel.get() * (1 - Math.pow(Math.E, -(Dashboard.functionROA.get()
    // * distance))) + (Dashboard.functionLRG.get() * Math.pow(distance, 3));
    if (distance < 2.0) {
      return 0.0;
    } else if (distance > 6.3) {
      return 45.0;
    } else {
      return distance * Dashboard.functionScalar.get();
    }
  }
}
