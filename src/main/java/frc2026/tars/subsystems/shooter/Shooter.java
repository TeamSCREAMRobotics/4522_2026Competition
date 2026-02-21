package frc2026.tars.subsystems.shooter;

import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.util.AllianceFlipUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2026.tars.RobotState;
import frc2026.tars.subsystems.shooter.flywheel.Flywheel;
import frc2026.tars.subsystems.shooter.hood.Hood;
import frc2026.tars.subsystems.shooter.turret.Turret;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends SubsystemBase {
  private final Flywheel flywheel;
  private final Hood hood;
  private final Turret turret;
  private final RobotState robotState;
  private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  private enum ShooterState {
    IDLE,
    STOWED,
    SHOOTING,
    FERRYING
  }

  @Getter @Setter private ShooterState state = ShooterState.IDLE;

  public Shooter(Flywheel flywheel, Hood hood, Turret turret, RobotState robotState) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    this.robotState = robotState;
  }

  public void idleCase() {
    if (robotState.getArea().isEmpty()) {
      return;
    } else {
      switch (robotState.getArea().get()) {
        case ALLIANCEZONE:
          setAimingTarget(FieldConstants.Hub.hubCenter);
          break;
        case DEPOT_SIDE_NEUTRALZONE:
          setAimingTarget(AllianceFlipUtil.get(FieldConstants.AllianceZones.leftAllianceZone, FieldConstants.AllianceZones.oppRightAllianceZone));
          break;
        case OUTPOST_SIDE_NEUTRALZONE:
        setAimingTarget(AllianceFlipUtil.get(FieldConstants.AllianceZones.rightAllianceZone, FieldConstants.AllianceZones.oppLeftAllianceZone));
          break;
        case OTHERALLIANCEZONE:
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
              break;
            case STOWED:
              break;
            case SHOOTING:
              break;
            case FERRYING:
              break;

            default:
              break;
          }
        });
  }

  public void setAimingTarget(Translation2d target){

  }
}
