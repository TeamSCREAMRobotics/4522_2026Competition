package frc2026.tars.subsystems.shooter;

import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.physics.Trajectory;
import com.teamscreamrobotics.physics.Trajectory.GamePiece;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
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
  private static final double FEED_DURATION_SECONDS = 3.0;

  private static final double EXIT_VELOCITY_RETENTION = 0.8;

  public void hoodMapPoints() {
    // TODO: Add tuned points to hood maps

    hoodMapAllianceZone.put(0.0, 0.0);
    hoodMapAllianceZone.put(5.0, 10.0);
    hoodMapAllianceZone.put(11.0, 22.0);

    hoodMapNeutralZone.put(0.0, 0.0);
    hoodMapNeutralZone.put(5.0, 10.0);
    hoodMapNeutralZone.put(11.0, 22.0);
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
    return Length.fromMeters(getFieldToTurret().getDistance(target));
  }

  private void applyAimingSetpoints(
      Pose2d robotPose,
      ChassisSpeeds robotSpeeds,
      Translation2d target,
      InterpolatingDoubleTreeMap treeMap) {
    setTarget(target);
    double distanceMeters = getShotDistance(target).getMeters();
    double hoodAngleDeg = treeMap.get(distanceMeters);

    Trajectory.configure()
        .setGamePiece(GamePiece.FUEL)
        .setInitialHeight(ShooterConstants.HEIGHT)
        .setTargetHeight(Trajectory.HUB_HEIGHT)
        .setTargetDistance(distanceMeters)
        .setShotAngle(hoodAngleDeg);

    double flywheelSetpoint = Trajectory.getRequiredVelocity() / 4.0;

    turret.aimOnTheFly(target, robotPose, robotSpeeds, getTimeOfFlight());
    hood.moveToAngleCommand(Rotation2d.fromDegrees(hoodAngleDeg));
    flywheel.setSetpointVelocity(flywheelSetpoint);

    Logger.log(logPrefix + "Hood Angle", hoodAngleDeg);
    Logger.log(logPrefix + "Flywheel Velocity", flywheelSetpoint);
    Logger.log(logPrefix + "Shot Distance", distanceMeters);
  }

  private void startFeedIfNotRunning() {
    if (!isFeedActive) {
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
            hoodMapAllianceZone);
        setIdleState(IdleState.IDLE_HUB);
        break;
      case DEPOT_SIDE_NEUTRALZONE:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.leftAllianceZone,
                FieldConstants.AllianceZones.oppRightAllianceZone),
            hoodMapNeutralZone);
        setIdleState(IdleState.IDLE_FERRY_DEPOT);
        break;
      case OUTPOST_SIDE_NEUTRALZONE:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.rightAllianceZone,
                FieldConstants.AllianceZones.oppLeftAllianceZone),
            hoodMapNeutralZone);
        setIdleState(IdleState.IDLE_FERRY_OUTPOST);
        break;
      default:
        setIdleState(IdleState.NA);
        break;
    }
  }

  private void ferryCase(RobotState.Area area, Pose2d robotPose, ChassisSpeeds robotSpeeds) {
    if (area == null) return;

    switch (area) {
      case DEPOT_SIDE_NEUTRALZONE:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.leftAllianceZone,
                FieldConstants.AllianceZones.oppRightAllianceZone),
            hoodMapNeutralZone);
        startFeedIfNotRunning();
        break;
      case OUTPOST_SIDE_NEUTRALZONE:
        applyAimingSetpoints(
            robotPose,
            robotSpeeds,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.rightAllianceZone,
                FieldConstants.AllianceZones.oppLeftAllianceZone),
            hoodMapNeutralZone);
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
    return run(
        () -> {
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
              hood.moveToAngleCommand(Rotation2d.fromDegrees(0.0));
              flywheel.setSetpointVelocity(7.5);
              if (isFeedActive) stopFeed();
              setIdleState(IdleState.NA);
              break;

            case SHOOTING:
              applyAimingSetpoints(
                  robotPose, robotSpeeds, FieldConstants.Hub.hubCenter, hoodMapAllianceZone);
              startFeedIfNotRunning();
              setIdleState(IdleState.NA);
              break;

            case FERRYING:
              ferryCase(area, robotPose, robotSpeeds);
              setIdleState(IdleState.NA);
              break;

            default:
              break;
          }
        });
  }

  @Override
  public void periodic() {
    Logger.log("Shooter/Shooter State", getState().toString());
    Logger.log("Shooter/Idle State", getIdleState().toString());
  }

  public Translation2d getFieldToTurret() {
    Pose2d pose = robotPose;

    return pose.getTranslation()
        .plus(transform3dTo2dXY(VisionManager.robotToTurretFixed).getTranslation());
  }

  public double getTimeOfFlight() {
    double distance = robotPose.getTranslation().getDistance(target);
    double exitVelocity = 15.0; // Conversions.rpsToMPS(flywheel.getVelocity(),
    // FlywheelConstants.FLYWHEEL_CIRCUMFERENCE.getMeters(),
    // FlywheelConstants.FLYWHEEL_REDUCTION) * EXIT_VELOCITY_RETENTION;
    double exitAngle =
        Math.toRadians(45.0); // ScreamMath.mapRange(hood.getPosition(), HoodConstants.MIN_UNITS,
    // HoodConstants.MAX_UNITS, HoodConstants.HOOD_MIN_EXIT_ANGLE.getRadians(),
    // HoodConstants.HOOD_MAX_EXIT_ANGLE.getRadians());
    double horizontalVelocity = exitVelocity * Math.cos(exitAngle);

    return distance / horizontalVelocity;
  }

  public static Transform2d transform3dTo2dXY(Transform3d transform) {
    return new Transform2d(
        transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
  }
}
