package frc2026.tars.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;

import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.physics.Trajectory;
import com.teamscreamrobotics.physics.Trajectory.GamePiece;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2026.tars.RobotState;
import frc2026.tars.subsystems.shooter.flywheel.Flywheel;
import frc2026.tars.subsystems.shooter.flywheel.Flywheel.FlywheelGoal;
import frc2026.tars.subsystems.shooter.hood.Hood;
import frc2026.tars.subsystems.shooter.hood.Hood.HoodGoal;
import frc2026.tars.subsystems.shooter.indexer.Feeder;
import frc2026.tars.subsystems.shooter.indexer.Feeder.FeederGoal;
import frc2026.tars.subsystems.shooter.indexer.Spindexer;
import frc2026.tars.subsystems.shooter.indexer.Spindexer.SpindexerGoal;
import frc2026.tars.subsystems.shooter.turret.Turret;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
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
  private final InterpolatingDoubleTreeMap hoodMapAllainceZone = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodMapNeutralZone = new InterpolatingDoubleTreeMap();
  private final Supplier<Pose2d> robotPose;
  private final String logprefix = "Subsystems/Shooter/";

  public void hoodMapPoints() {
    // TODO: Add points to hood maps

    hoodMapAllainceZone.put(0.0, 0.0);
    hoodMapAllainceZone.put(5., 10.0);
    hoodMapAllainceZone.put(10.0, 20.0);

    hoodMapNeutralZone.put(0.0, 0.0);
    hoodMapNeutralZone.put(0.5, 10.0);
    hoodMapNeutralZone.put(1.0, 20.0);
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

  public Distance getShotDistance(Translation2d target) {
    Pose2d robotPose = this.robotPose.get();
    double centerToTargetMeters = robotPose.getTranslation().getDistance(target);

    double centerToShooterMeters = 0.146;

    double shooterToTargetMeters =
        Math.sqrt(Math.pow(centerToTargetMeters, 2.0) - Math.pow(centerToShooterMeters, 2.0));

    return Units.Meters.of(shooterToTargetMeters);
  }

  public Command aimAtPoint(InterpolatingDoubleTreeMap treeMap, double distance) {
    return run(
        () -> {
          Hood.angle = () -> treeMap.get(distance);
          hood.applyGoalCommand(HoodGoal.TOPOSE);
        });
  }

  public Command rampUpToVelocity(double velocity) {
    return run(
        () -> {
          Flywheel.shootVel = () -> velocity;
          flywheel.applyGoalCommand(FlywheelGoal.SHOOTING);
        });
  }

  public Command feed(BooleanSupplier end) {
    return spindexer
        .applyGoalCommand(SpindexerGoal.RUN)
        .alongWith(feeder.applyGoalCommand(FeederGoal.RUN))
        .until(end)
        .andThen(
            spindexer
                .applyGoalCommand(SpindexerGoal.STOP)
                .alongWith(feeder.applyGoalCommand(FeederGoal.STOP)));
  }

  public void setAimingTarget(InterpolatingDoubleTreeMap treeMap, Translation2d target) {
    DoubleSupplier distance = () -> getShotDistance(target).in(Meters);
    Trajectory.configure()
        .setGamePiece(GamePiece.FUEL)
        .setInitialHeight(ShooterConstants.HEIGHT)
        .setTargetHeight(Trajectory.HUB_HEIGHT)
        .setTargetDistance(distance.getAsDouble())
        .setShotAngle(treeMap.get(distance.getAsDouble()));

    aimAtPoint(treeMap, distance.getAsDouble());
    rampUpToVelocity(Trajectory.getRequiredVelocity() / 4);

    Logger.log(logprefix + "Hood Angle", treeMap.get(distance.getAsDouble()));
    Logger.log(logprefix + "Flywheel Velocity", Trajectory.getRequiredVelocity() / 4);
  }

  public void setAimingTarget(
      InterpolatingDoubleTreeMap treeMap, Translation2d target, boolean feed) {
    DoubleSupplier distance = () -> getShotDistance(target).in(Meters);
    Trajectory.configure()
        .setGamePiece(GamePiece.FUEL)
        .setInitialHeight(ShooterConstants.HEIGHT)
        .setTargetHeight(Trajectory.HUB_HEIGHT)
        .setTargetDistance(distance.getAsDouble())
        .setShotAngle(treeMap.get(distance.getAsDouble()));

    aimAtPoint(treeMap, distance.getAsDouble());
    rampUpToVelocity(Trajectory.getRequiredVelocity() / 4);
    if (feeder.atGoal()) {
      double time = Timer.getFPGATimestamp();
      feed(() -> time >= 3.0);
    } else return;
  }

  public void idleCase() {
    if (robotState.getArea().isEmpty()) {
      return;
    } else {
      switch (robotState.getArea().get()) {
        case ALLIANCEZONE:
          setAimingTarget(hoodMapAllainceZone, FieldConstants.Hub.hubCenter);
          break;
        case DEPOT_SIDE_NEUTRALZONE:
          setAimingTarget(
              hoodMapNeutralZone,
              AllianceFlipUtil.get(
                  FieldConstants.AllianceZones.leftAllianceZone,
                  FieldConstants.AllianceZones.oppRightAllianceZone));
          break;
        case OUTPOST_SIDE_NEUTRALZONE:
          setAimingTarget(
              hoodMapNeutralZone,
              AllianceFlipUtil.get(
                  FieldConstants.AllianceZones.rightAllianceZone,
                  FieldConstants.AllianceZones.oppLeftAllianceZone));
          break;
        case OTHERALLIANCEZONE:
          break;
        default:
          break;
      }
    }
  }

  public void ferryCase() {
    if (robotState.getArea().isEmpty()) {
      return;
    } else {
      switch (robotState.getArea().get()) {
        case DEPOT_SIDE_NEUTRALZONE:
          setAimingTarget(
              hoodMapNeutralZone,
              AllianceFlipUtil.get(
                  FieldConstants.AllianceZones.leftAllianceZone,
                  FieldConstants.AllianceZones.oppRightAllianceZone),
              true);
          break;
        case OUTPOST_SIDE_NEUTRALZONE:
          setAimingTarget(
              hoodMapNeutralZone,
              AllianceFlipUtil.get(
                  FieldConstants.AllianceZones.rightAllianceZone,
                  FieldConstants.AllianceZones.oppLeftAllianceZone),
              true);
          break;
        default:
          break;
      }
    }
  }

  public Command defaultCommand() {
    return run(
        () -> {
          switch (state) {
            case IDLE:
              idleCase();
              break;
            case STOWED:
              hood.applyGoalCommand(HoodGoal.ZERO);
              flywheel.applyGoalCommand(FlywheelGoal.IDLE);
              break;
            case SHOOTING:
              setAimingTarget(hoodMapAllainceZone, FieldConstants.Hub.hubCenter, true);
              break;
            case FERRYING:
              ferryCase();
              break;
            default:
              break;
          }
        });
  }

  @Override
  public void periodic() {}
}
