package frc2026.tars.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;

import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.physics.Trajectory;
import com.teamscreamrobotics.physics.Trajectory.GamePiece;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2026.tars.RobotState;
import frc2026.tars.controlboard.Controlboard;
import frc2026.tars.subsystems.shooter.flywheel.Flywheel;
import frc2026.tars.subsystems.shooter.hood.Hood;
import frc2026.tars.subsystems.shooter.indexer.Feeder;
import frc2026.tars.subsystems.shooter.indexer.Feeder.FeederGoal;
import frc2026.tars.subsystems.shooter.indexer.Spindexer;
import frc2026.tars.subsystems.shooter.indexer.Spindexer.SpindexerGoal;
import frc2026.tars.subsystems.shooter.turret.Turret;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends SubsystemBase {
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;
  private final Spindexer spindexer;
  private final Feeder feeder;
  private final RobotState robotState;
  private final InterpolatingDoubleTreeMap hoodMapAllianceZone = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodMapNeutralZone = new InterpolatingDoubleTreeMap();
  private final Supplier<Pose2d> robotPose;
  private final String logPrefix = "Subsystems/Shooter/";

  // Feed state tracking — avoids spawning a new Command every loop
  private boolean isFeedActive = false;
  private final Timer feedTimer = new Timer();
  private static final double FEED_DURATION_SECONDS = 3.0;

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

  @Getter @Setter private ShooterState state = ShooterState.IDLE;

  public Shooter(
      Flywheel flywheel,
      Hood hood,
      Turret turret,
      Spindexer spindexer,
      Feeder feeder,
      Supplier<Pose2d> robotPose,
      RobotState robotState) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    this.spindexer = spindexer;
    this.feeder = feeder;
    this.robotPose = robotPose;
    this.robotState = robotState;

    hoodMapPoints();
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------

  public Distance getShotDistance(Translation2d target) {
    Pose2d pose = this.robotPose.get();
    double centerToTarget = pose.getTranslation().getDistance(target);
    double centerToShooter = 0.146; // meters
    // Pythagorean offset so we measure from the shooter, not the robot center
    double shooterToTarget =
        Math.sqrt(Math.pow(centerToTarget, 2.0) - Math.pow(centerToShooter, 2.0));
    return Units.Meters.of(shooterToTarget);
  }

  // ---------------------------------------------------------------------------
  // Core aiming — called directly each loop, no Command wrappers
  // ---------------------------------------------------------------------------

  /**
   * Configures the hood angle and flywheel velocity for a given target. Must be called from inside
   * a periodic/run context, NOT used to produce a Command object.
   */
  private void applyAimingSetpoints(InterpolatingDoubleTreeMap treeMap, Translation2d target) {
    double distanceMeters = getShotDistance(target).in(Meters);

    Trajectory.configure()
        .setGamePiece(GamePiece.FUEL)
        .setInitialHeight(ShooterConstants.HEIGHT)
        .setTargetHeight(Trajectory.HUB_HEIGHT)
        .setTargetDistance(distanceMeters)
        .setShotAngle(treeMap.get(distanceMeters));

    double hoodAngleDeg = treeMap.get(distanceMeters);
    // Trajectory.getRequiredVelocity() returns surface speed; divide by 4 for motor setpoint
    double flywheelSetpoint = Trajectory.getRequiredVelocity() / 4.0;

    hood.moveToAngleCommand(Rotation2d.fromDegrees(hoodAngleDeg));
    flywheel.setSetpointVelocity(flywheelSetpoint);

    Logger.log(logPrefix + "Hood Angle", hoodAngleDeg);
    Logger.log(logPrefix + "Flywheel Velocity", flywheelSetpoint);
    Logger.log(logPrefix + "Shot Distance", distanceMeters);
  }

  // ---------------------------------------------------------------------------
  // Feed management — timer-based, no repeated Command construction
  // ---------------------------------------------------------------------------

  /** Starts a timed feed cycle. Safe to call every loop; only arms once per press. */
  private void startFeedIfNotRunning() {
    if (!isFeedActive) {
      isFeedActive = true;
      feedTimer.reset();
      feedTimer.start();
      spindexer.applyGoal(SpindexerGoal.RUN);
      feeder.applyGoal(FeederGoal.RUN);
    }
  }

  /** Stops feeding and resets state. */
  private void stopFeed() {
    isFeedActive = false;
    feedTimer.stop();
    feedTimer.reset();
    spindexer.applyGoal(SpindexerGoal.STOP);
    feeder.applyGoal(FeederGoal.STOP);
  }

  /** Must be called each loop to time out the feed cycle. */
  private void updateFeed() {
    if (isFeedActive && feedTimer.hasElapsed(FEED_DURATION_SECONDS)) {
      stopFeed();
    }
  }

  // ---------------------------------------------------------------------------
  // State cases
  // ---------------------------------------------------------------------------

  private void idleCase() {
    if (robotState.getArea().isEmpty()) return;

    switch (robotState.getArea().get()) {
      case ALLIANCEZONE:
        applyAimingSetpoints(hoodMapAllianceZone, FieldConstants.Hub.hubCenter);
        break;
      case DEPOT_SIDE_NEUTRALZONE:
        applyAimingSetpoints(
            hoodMapNeutralZone,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.leftAllianceZone,
                FieldConstants.AllianceZones.oppRightAllianceZone));
        break;
      case OUTPOST_SIDE_NEUTRALZONE:
        applyAimingSetpoints(
            hoodMapNeutralZone,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.rightAllianceZone,
                FieldConstants.AllianceZones.oppLeftAllianceZone));
        break;
      default:
        break;
    }
  }

  private void ferryCase() {
    if (robotState.getArea().isEmpty()) return;

    switch (robotState.getArea().get()) {
      case DEPOT_SIDE_NEUTRALZONE:
        applyAimingSetpoints(
            hoodMapNeutralZone,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.leftAllianceZone,
                FieldConstants.AllianceZones.oppRightAllianceZone));
        startFeedIfNotRunning();
        break;
      case OUTPOST_SIDE_NEUTRALZONE:
        applyAimingSetpoints(
            hoodMapNeutralZone,
            AllianceFlipUtil.get(
                FieldConstants.AllianceZones.rightAllianceZone,
                FieldConstants.AllianceZones.oppLeftAllianceZone));
        startFeedIfNotRunning();
        break;
      default:
        stopFeed();
        break;
    }
  }

  // ---------------------------------------------------------------------------
  // State machine update
  // ---------------------------------------------------------------------------

  private void updateShooterState() {
    if (robotState.getArea().isEmpty()) return;

    RobotState.Area area = robotState.getArea().get();

    if (Controlboard.shoot().getAsBoolean() && area == RobotState.Area.ALLIANCEZONE) {
      setState(ShooterState.SHOOTING);
    } else if (Controlboard.shoot().getAsBoolean()
        && (area == RobotState.Area.DEPOT_SIDE_NEUTRALZONE
            || area == RobotState.Area.OUTPOST_SIDE_NEUTRALZONE)) {
      setState(ShooterState.FERRYING);
    } else if (area == RobotState.Area.TRENCHES) {
      setState(ShooterState.STOWED);
    } else {
      setState(ShooterState.IDLE);
    }
  }

  // ---------------------------------------------------------------------------
  // Default command — one run() wrapper; all logic inside uses direct calls
  // ---------------------------------------------------------------------------

  public Command defaultCommand() {
    return run(
        () -> {
          updateShooterState();
          updateFeed(); // advance the feed timer every loop

          switch (state) {
            case IDLE:
              idleCase();
              break;

            case STOWED:
              // Park the hood flat and spin the flywheel at a slow keep-warm speed
              hood.moveToAngleCommand(Rotation2d.fromDegrees(0.0));
              flywheel.setSetpointVelocity(7.5);
              if (isFeedActive) stopFeed();
              break;

            case SHOOTING:
              applyAimingSetpoints(hoodMapAllianceZone, FieldConstants.Hub.hubCenter);
              startFeedIfNotRunning();
              break;

            case FERRYING:
              ferryCase();
              break;

            default:
              break;
          }
        });
  }
}
