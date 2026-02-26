package frc2026.tars.subsystems.shooter;

import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.gameutil.FieldConstants;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2026.tars.RobotState;
import frc2026.tars.controlboard.Controlboard;
import frc2026.tars.subsystems.drivetrain.Drivetrain;
import frc2026.tars.subsystems.shooter.flywheel.Flywheel;
import frc2026.tars.subsystems.shooter.hood.Hood;
import frc2026.tars.subsystems.shooter.indexer.Feeder;
import frc2026.tars.subsystems.shooter.indexer.Feeder.FeederGoal;
import frc2026.tars.subsystems.shooter.indexer.Spindexer;
import frc2026.tars.subsystems.shooter.indexer.Spindexer.SpindexerGoal;
import frc2026.tars.subsystems.shooter.turret.Turret;
import frc2026.tars.subsystems.vision.VisionManager;
import frc2026.tars.util.Util;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends SubsystemBase {
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;
  private final Spindexer spindexer;
  private final Feeder feeder;
  private final Drivetrain drivetrain;
  private final RobotState robotState;
  private Pose2d robotPose;
  private ChassisSpeeds robotSpeeds;
  private static final InterpolatingDoubleTreeMap hoodMapAllianceZone =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap hoodMapNeutralZone =
      new InterpolatingDoubleTreeMap();
  private final String logPrefix = "Subsystems/Shooter/";

  @Getter @Setter private Translation2d target = new Translation2d();

  private boolean wantShoot = false;

  // Feed state tracking — avoids spawning a new Command every loop
  private boolean isFeedActive = false;
  private final Timer feedTimer = new Timer();
  private static final double FEED_DURATION_SECONDS = 1.0;

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

  private enum ShooterState {
    IDLE,
    STOWED,
    SHOOTING,
    FERRYING
  }

  private enum IdleState {
    NA,
    IDLE_HUB,
    IDLE_FERRY_DEPOT,
    IDLE_FERRY_OUTPOST,
  }

  @Getter @Setter private ShooterState state = ShooterState.IDLE;
  @Getter @Setter private IdleState idleState = IdleState.NA;

  public Shooter(
      Flywheel flywheel,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Feeder feeder,
      Drivetrain drivetrain,
      RobotState robotState) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    this.spindexer = spindexer;
    this.feeder = feeder;
    this.drivetrain = drivetrain;
    this.robotState = robotState;

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

  private void applyAimingSetpoints(
      Pose2d robotPose,
      ChassisSpeeds robotSpeeds,
      Translation2d target,
      InterpolatingDoubleTreeMap treeMap,
      boolean wantShoot) {
    setTarget(target);
    double distanceMeters = getShotDistance(target).getMeters();
    double hoodAngleDeg = treeMap.get(distanceMeters);

    /* Trajectory.configure()
        .setGamePiece(GamePiece.FUEL)
        .setInitialHeight(ShooterConstants.HEIGHT)
        .setTargetHeight(Trajectory.HUB_HEIGHT)
        .setTargetDistance(distanceMeters)
        .setShotAngle(90.0 - (hoodAngleDeg + HoodConstants.HOOD_OFFSET.getDegrees()));

    double multiplier = wantShoot ? 1.0 : 4.0;

    double flywheelSetpoint =
        (Conversions.mpsToRPS(
                Trajectory.getRequiredVelocity(),
                FlywheelConstants.FLYWHEEL_CIRCUMFERENCE.getMeters(),
                FlywheelConstants.FLYWHEEL_REDUCTION)) * 2.0; */

    double flywheelSetpoint = distanceMeters * SmartDashboard.getNumber("Distance Coeff", 1.0);

    // turret.aimOnTheFly(target, robotPose, robotSpeeds, Trajectory.getTimeOfFlight());
    turret.pointToTargetFR(() -> target, () -> robotPose);

    hood.moveToAngle(Rotation2d.fromDegrees(hoodAngleDeg));
    flywheel.setTargetVelocityTorqueCurrent(flywheelSetpoint, 0.0);

    Logger.log(logPrefix + "Hood Angle", hoodAngleDeg);
    Logger.log(logPrefix + "Flywheel Velocity", flywheelSetpoint);
    Logger.log(logPrefix + "Shot Distance", distanceMeters);
  }

  private void startFeedIfNotRunning() {
    if (!isFeedActive && flywheel.atVel()) {
      isFeedActive = true;
      feedTimer.reset();
      feedTimer.start();
      spindexer.applyGoal(SpindexerGoal.RUN);
      feeder.applyGoal(FeederGoal.RUN);
    }
  }

  private void stopFeed() {
    isFeedActive = false;
    feedTimer.stop();
    feedTimer.reset();
    spindexer.applyGoal(SpindexerGoal.STOP);
    feeder.applyGoal(FeederGoal.STOP);
  }

  private void updateFeed() {
    if (isFeedActive && feedTimer.hasElapsed(FEED_DURATION_SECONDS)) {
      stopFeed();
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
            hoodMapAllianceZone,
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
            hoodMapNeutralZone,
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
            hoodMapNeutralZone,
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
            hoodMapNeutralZone,
            wantShoot);
        startFeedIfNotRunning();
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
        startFeedIfNotRunning();
        break;
      default:
        stopFeed();
        break;
    }
  }

  private void updateShooterState(RobotState.Area area, boolean wantShoot) {
    if (wantShoot && area == RobotState.Area.ALLIANCEZONE) {
      setState(ShooterState.SHOOTING);
    } else if (wantShoot
        && (area == RobotState.Area.DEPOT_SIDE_NEUTRALZONE
            || area == RobotState.Area.OUTPOST_SIDE_NEUTRALZONE)) {
      setState(ShooterState.FERRYING);
    } else if (area == RobotState.Area.TRENCHES) {
      setState(ShooterState.STOWED);
    } else {
      setState(ShooterState.IDLE);
    }
  }

  public Command defaultCommand() {
    return run(() -> {
          RobotState.Area area = robotState.getArea();
          robotPose = drivetrain.getEstimatedPose();
          robotSpeeds = drivetrain.getState().Speeds;
          wantShoot = Controlboard.shoot().getAsBoolean();
          updateShooterState(area, wantShoot);
          updateFeed();

          switch (state) {
            case IDLE:
              idleCase(area, robotPose, robotSpeeds);
              break;

            case STOWED:
              hood.moveToAngle(Rotation2d.fromDegrees(0.0));
              flywheel.setTargetVelocityTorqueCurrent(7.5, 0);
              if (isFeedActive) stopFeed();
              setIdleState(IdleState.NA);
              break;

            case SHOOTING:
              applyAimingSetpoints(
                  robotPose,
                  robotSpeeds,
                  AllianceFlipUtil.get(
                      FieldConstants.Hub.hubCenter, FieldConstants.Hub.oppHubCenter),
                  hoodMapAllianceZone,
                  true);
              startFeedIfNotRunning();
              setIdleState(IdleState.NA);
              break;

            case FERRYING:
              ferryCase(area, robotPose, robotSpeeds, wantShoot);
              setIdleState(IdleState.NA);
              break;

            default:
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

  public double getTimeOfFlight(double velocity) {
    double distance = robotPose.getTranslation().getDistance(target);
    double exitVelocity = velocity; // Conversions.rpsToMPS(flywheel.getVelocity(),
    // FlywheelConstants.FLYWHEEL_CIRCUMFERENCE.getMeters(),
    // FlywheelConstants.FLYWHEEL_REDUCTION) * EXIT_VELOCITY_RETENTION;
    double exitAngle =
        Math.toRadians(45.0); // ScreamMath.mapRange(hood.getPosition(), HoodConstants.MIN_UNITS,
    // HoodConstants.MAX_UNITS, HoodConstants.HOOD_MIN_EXIT_ANGLE.getRadians(),
    // HoodConstants.HOOD_MAX_EXIT_ANGLE.getRadians());
    double horizontalVelocity = exitVelocity * Math.cos(exitAngle);

    return distance / horizontalVelocity;
  }
}
