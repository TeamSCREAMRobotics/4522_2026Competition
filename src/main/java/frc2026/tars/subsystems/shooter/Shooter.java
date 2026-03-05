package frc2026.tars.subsystems.shooter;

import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.math.Conversions;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.GeomUtil;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2026.tars.RobotState;
import frc2026.tars.controlboard.Controlboard;
import frc2026.tars.controlboard.Dashboard;
import frc2026.tars.subsystems.drivetrain.Drivetrain;
import frc2026.tars.subsystems.intake.IntakeRollers;
import frc2026.tars.subsystems.intake.IntakeRollers.IntakeRollersGoal;
import frc2026.tars.subsystems.intake.IntakeWrist;
import frc2026.tars.subsystems.intake.IntakeWrist.IntakeWristGoal;
import frc2026.tars.subsystems.leds.LED;
import frc2026.tars.subsystems.shooter.dyerotor.Dyerotor;
import frc2026.tars.subsystems.shooter.flywheel.Flywheel;
import frc2026.tars.subsystems.shooter.flywheel.FlywheelConstants;
import frc2026.tars.subsystems.shooter.hood.Hood;
import frc2026.tars.subsystems.shooter.hood.HoodConstants;
import frc2026.tars.subsystems.shooter.turret.Turret;
import frc2026.tars.subsystems.vision.VisionManager;
import frc2026.tars.util.Util;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends SubsystemBase {
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;
  private final Dyerotor dyerotor;
  private final Drivetrain drivetrain;
  private final RobotState robotState;
  private final IntakeWrist intakeWrist;
  private final IntakeRollers intakeRollers;
  private final LED led;
  private Pose2d robotPose;
  private ChassisSpeeds robotSpeeds;
  private static final InterpolatingDoubleTreeMap hoodMapAllianceZone =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodMapNeutralZone =
      new InterpolatingDoubleTreeMap();
  private final String logPrefix = "Subsystems/Shooter/";

  @Getter @Setter private Translation2d target = new Translation2d();

  private boolean wantShoot = false;

  public void hoodMapPoints() {
    // TODO: Add tuned points to hood maps

    hoodMapAllianceZone.put(0.0, 0.0);
    hoodMapAllianceZone.put(5.0, 10.0);
    hoodMapAllianceZone.put(11.0, 22.0);

    hoodMapNeutralZone.put(0.0, 0.0);
    hoodMapNeutralZone.put(5.0, 10.0);
    hoodMapNeutralZone.put(11.0, 22.0);

    SmartDashboard.putNumber("Distance Coeff", 1.0);
  }

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
      Turret turret,
      Dyerotor dyerotor,
      IntakeWrist intakeWrist,
      IntakeRollers intakeRollers,
      LED led,
      Drivetrain drivetrain,
      RobotState robotState) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    this.dyerotor = dyerotor;
    this.intakeWrist = intakeWrist;
    this.intakeRollers = intakeRollers;
    this.drivetrain = drivetrain;
    this.robotState = robotState;
    this.led = led;

    hoodMapPoints();
  }

  public Length getShotDistance(Translation2d target) {
    /* Pose2d pose = this.robotPose.get();
    double centerToTarget = pose.getTranslation().getDistance(target);
    double centerToShooter = 0.146; // meters
    // Pythagorean offset so we measure from the shooter, not the robot center
    double shooterToTarget =
        Math.sqrt(Math.pow(centerToTarget, 2.0) - Math.pow(centerToShooter, 2.0));
    return Length.fromMeters(shooterToTarget); */
    return Length.fromMeters(getFieldToTurret().getTranslation().getDistance(target));
  }

  private void shootOnTheFly(
      Pose2d robotPose, ChassisSpeeds robotSpeed, Translation2d target, boolean wantShoot) {

    double distance = getShotDistance(target).getMeters();

    double flywheelVelocity = ShooterConstants.FLYWHEEL_MAP.get(distance);

    double hoodAngle = getHoodAngleFromDistance(distance) + HoodConstants.HOOD_OFFSET.getDegrees();

    double timeOfFlight =
        getTimeOfFlight(
            robotPose.getTranslation().getDistance(target), flywheelVelocity, hoodAngle);

    Translation2d futurePos =
        getFieldToTurret()
            .getTranslation()
            .plus(
                new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
                    .times(ShooterConstants.LATENCY + timeOfFlight));

    // double multiplier = wantShoot ? 1.0 : 8.0;

    double futureDistance = futurePos.getDistance(target);
    double futureVelocity = ShooterConstants.FLYWHEEL_MAP.get(futureDistance);

    Translation2d futureTargetVec = target.minus(futurePos);

    double turretAngleField = futureTargetVec.getAngle().getDegrees();

    turret.moveToAngleFR(Rotation2d.fromDegrees(turretAngleField), robotPose.getRotation());

    hood.moveToAngle(Rotation2d.fromDegrees(getHoodAngleFromDistance(futureDistance)));

    flywheel.setTargetVelocityTorqueCurrent(futureVelocity, 0.0);

    Logger.log("SOTM/ToF", timeOfFlight);
    Logger.log("SOTM/FuturePose", new Pose2d(futurePos, robotPose.getRotation()));
    Logger.log("SOTM/FutureDistance", futureDistance);
    Logger.log("SOTM/TurretAngle", turretAngleField);
  }

  private void applyAimingSetpoints(
      Pose2d robotPose,
      ChassisSpeeds robotSpeeds,
      Translation2d target,
      InterpolatingDoubleTreeMap treeMap,
      boolean wantShoot) {
    setTarget(target);
    double distanceMeters = getShotDistance(target).getMeters();
    double hoodAngleDeg = getHoodAngleFromDistance(distanceMeters);

    double multiplier = wantShoot ? 1.0 : 8.0;
    double flywheelMap = ShooterConstants.FLYWHEEL_MAP.get(distanceMeters / multiplier);

    double flywheelSetpoint = flywheelMap;

    if (distanceMeters <= 2.0) {
      flywheelSetpoint = flywheelMap * Dashboard.closeMapNudge.get();
    } else if (distanceMeters <= 4.0 && distanceMeters > 2.0) {
      flywheelSetpoint = flywheelMap * Dashboard.midMapNudge.get();
    } else flywheelSetpoint = flywheelMap * Dashboard.farMapNudge.get();

    turret.pointToTargetFR(() -> target, () -> robotPose, () -> getFieldToTurret());

    hood.moveToAngle(Rotation2d.fromDegrees(hoodAngleDeg));
    flywheel.setTargetVelocityTorqueCurrent(flywheelSetpoint, 0.0);
    // flywheel.setTargetVelocityTorqueCurrent(Dashboard.flywheelVelocity.get(), 0.0);

    Logger.log(logPrefix + "Hood Angle", hoodAngleDeg);
    Logger.log(logPrefix + "Flywheel Velocity", flywheelSetpoint);
    Logger.log(logPrefix + "Shot Distance", distanceMeters);
  }

  public Command autoShoot(double time) {
    return Commands.run(
            () -> {
              wantShoot = true;
            })
        .withTimeout(time)
        .finallyDo(() -> wantShoot = false);
  }

  public void runFeed() {
    if ((flywheel.atVel() || Dashboard.disableWaitUntilAtVelocity.get())) {
      dyerotor.runDyerotor();
      led.strobe(Color.kRed, 0.7);
    }
  }

  public void stopFeed() {
    dyerotor.stop();
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
      intakeWrist.applyGoal(IntakeWristGoal.AGITATE);
    } else {
      intakeWrist.applyGoal(IntakeWristGoal.EXTENDED);
    }
  }

  private void idleCase(RobotState.Area area, Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    if (area == null) return;
    if (Dashboard.disableShootOnTheMove.get()) {
      switch (area) {
        case ALLIANCEZONE:
          applyAimingSetpoints(
              robotPose,
              robotSpeeds,
              AllianceFlipUtil.get(FieldConstants.Hub.hubCenter, FieldConstants.Hub.oppHubCenter),
              hoodMapAllianceZone,
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
              hoodMapNeutralZone,
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
              hoodMapNeutralZone,
              wantShoot);
          setIdleState(IdleState.IDLE_FERRY_OUTPOST);
          led.wave(Color.kBlack, new Color(0.0f, 0.5019608f, 0.5019608f), 0.1, 1.25);
          break;
        case OTHERALLIANCEZONE:
          applyAimingSetpoints(
              robotPose,
              robotSpeeds,
              AllianceFlipUtil.get(
                  FieldConstants.AllianceZones.rightAllianceZone,
                  FieldConstants.AllianceZones.oppLeftAllianceZone),
              hoodMapNeutralZone,
              wantShoot);
          led.wave(
              Color.kBlack,
              AllianceFlipUtil.get(
                  new Color(0.26078432f, 1.0f, 0.36078432f) /*new Color(0.0f, 1.0f, 0.83137256f)*/,
                  new Color(1.0f, 0.49803922f, 0.83137256f)),
              0.1,
              1.25);
        default:
          setIdleState(IdleState.NA);
          break;
      }
    } else {
      switch (area) {
        case ALLIANCEZONE:
          shootOnTheFly(
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
          shootOnTheFly(
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
          shootOnTheFly(
              robotPose,
              robotSpeeds,
              AllianceFlipUtil.get(
                  FieldConstants.AllianceZones.rightAllianceZone,
                  FieldConstants.AllianceZones.oppLeftAllianceZone),
              wantShoot);
          setIdleState(IdleState.IDLE_FERRY_OUTPOST);
          led.wave(Color.kBlack, new Color(0.0f, 0.5019608f, 0.5019608f), 0.1, 1.25);
          break;
        case OTHERALLIANCEZONE:
          applyAimingSetpoints(
              robotPose,
              robotSpeeds,
              AllianceFlipUtil.get(
                  FieldConstants.AllianceZones.rightAllianceZone,
                  FieldConstants.AllianceZones.oppLeftAllianceZone),
              hoodMapNeutralZone,
              wantShoot);
          led.wave(
              Color.kBlack,
              AllianceFlipUtil.get(
                  new Color(0.26078432f, 1.0f, 0.36078432f) /*new Color(0.0f, 1.0f, 0.83137256f)*/,
                  new Color(1.0f, 0.49803922f, 0.83137256f)),
              0.1,
              1.25);
        default:
          setIdleState(IdleState.NA);
          break;
      }
    }
  }

  private void ferryCase(
      RobotState.Area area, Pose2d robotPose, ChassisSpeeds robotSpeeds, boolean wantShoot) {
    if (area == null) return;

    if (Dashboard.disableShootOnTheMove.get()) {
      switch (area) {
        case DEPOT_SIDE_NEUTRALZONE:
          applyAimingSetpoints(
              robotPose,
              robotSpeeds,
              AllianceFlipUtil.get(
                  FieldConstants.AllianceZones.leftAllianceZone,
                  FieldConstants.AllianceZones.oppRightAllianceZone),
              hoodMapNeutralZone,
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
              hoodMapNeutralZone,
              wantShoot);
          runFeed();
          break;
        default:
          stopFeed();
          break;
      }
    } else {
      switch (area) {
        case DEPOT_SIDE_NEUTRALZONE:
          shootOnTheFly(
              robotPose,
              robotSpeeds,
              AllianceFlipUtil.get(
                  FieldConstants.AllianceZones.leftAllianceZone,
                  FieldConstants.AllianceZones.oppRightAllianceZone),
              wantShoot);
          runFeed();
          break;
        case OUTPOST_SIDE_NEUTRALZONE:
          shootOnTheFly(
              robotPose,
              robotSpeeds,
              AllianceFlipUtil.get(
                  FieldConstants.AllianceZones.rightAllianceZone,
                  FieldConstants.AllianceZones.oppLeftAllianceZone),
              wantShoot);
          runFeed();
          break;
        default:
          stopFeed();
          break;
      }
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
            || area == RobotState.Area.OTHERALLIANCEZONE)) {
      setState(ShooterState.FERRYING);
    } else if (area == RobotState.Area.TRENCHES) {
      setState(ShooterState.STOWED);
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

          updateShooterState(area, wantShoot);

          switch (state) {
            case IDLE:
              idleCase(area, robotPose, robotSpeeds);
              agitateStartTime = 0.0;
              agitateForward = true;
              stopFeed();

              break;

            case STOWED:
              hood.setVoltage(-2.2);
              flywheel.setTargetVelocityTorqueCurrent(7.5, 0);
              stopFeed();
              agitateStartTime = 0.0;
              agitateForward = true;

              led.rainbow(3.0, 1.0);

              setIdleState(IdleState.NA);
              break;

            case SHOOTING:
              if (Dashboard.disableShootOnTheMove.get()) {
                applyAimingSetpoints(
                    robotPose,
                    robotSpeeds,
                    AllianceFlipUtil.get(
                        FieldConstants.Hub.hubCenter, FieldConstants.Hub.oppHubCenter),
                    hoodMapAllianceZone,
                    wantShoot);
                runFeed();
                setIdleState(IdleState.NA);
                break;
              } else {
                shootOnTheFly(
                    robotPose,
                    robotSpeeds,
                    AllianceFlipUtil.get(
                        FieldConstants.Hub.hubCenter, FieldConstants.Hub.oppHubCenter),
                    wantShoot);
                setIdleState(IdleState.NA);
                runFeed();
                break;
              }
            case FERRYING:
              ferryCase(area, robotPose, robotSpeeds, wantShoot);
              setIdleState(IdleState.NA);
              agitateStartTime = 0.0;
              agitateForward = false;
              break;

            case INTAKE_UP:
              turret.moveToAngleRR(Rotation2d.fromDegrees(90.0));
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
          new Pose3d(getFieldToTurret().getX(), getFieldToTurret().getY(), 0.5, Rotation3d.kZero));
    }
  }

  public Pose2d getFieldToTurret() {
    Pose2d pose = robotPose;

    return GeomUtil.transformToPose(
        GeomUtil.poseToTransform(pose)
            .plus(Util.transform3dTo2dXY(VisionManager.robotToTurretFixed)));
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

    return distance / horizontalVelocity;
  }

  public double getHoodAngleFromDistance(double distance) {
    // return Dashboard.saturationLevel.get() * (1 - Math.pow(Math.E, -(Dashboard.functionROA.get()
    // * distance))) + (Dashboard.functionLRG.get() * Math.pow(distance, 3));
    if (distance < 2.0) {
      return 0.0;
    } else if (distance > 6.0) {
      return 20.0;
    } else {
      return (Dashboard.functionCurve.get() * Math.pow(distance, 3))
          * Dashboard.functionScalar.get();
    }
  }
}
