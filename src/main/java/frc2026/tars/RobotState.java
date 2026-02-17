package frc2026.tars;

import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.gameutil.GameState;
import com.teamscreamrobotics.util.Logger;
import com.teamscreamrobotics.zones.RectangularPoseArea;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc2026.tars.RobotContainer.Subsystems;
import frc2026.tars.controlboard.Controlboard;
import frc2026.tars.subsystems.drivetrain.Drivetrain;
import frc2026.tars.subsystems.intake.IntakeWrist;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class RobotState {
  private final Drivetrain drivetrain;
  private final IntakeWrist intakeWrist;
  private static final RectangularPoseArea test =
      new RectangularPoseArea(new Translation2d(1, 1), new Translation2d(5, 5));

  public enum Mode {
    AUTO,
    TELEOP,
    SHOOTINGTOHUB,
    FERRYING,
    CLIMBING,
    NOTHING
  }

  public enum Area {
    REDALLIANCE(FieldConstants.AllianceZones.redAlliance),
    BLUEALLIANCE(FieldConstants.AllianceZones.blueAlliance),
    BUMPS(
        FieldConstants.LeftBump.leftBump,
        FieldConstants.LeftBump.oppLeftBump,
        FieldConstants.RightBump.rightBump,
        FieldConstants.RightBump.oppRightBump),
    TRENCHES(
        FieldConstants.LeftTrench.leftTrench,
        FieldConstants.LeftTrench.oppLeftTrench,
        FieldConstants.RightTrench.rightTrench,
        FieldConstants.RightTrench.oppRightTrench),
    UPPERNEUTRALZONE(FieldConstants.NeutralZones.UPPER_NEUTRAL),
    LOWERNEUTRALZONE(FieldConstants.NeutralZones.LOWER_NEUTRAL);

    public List<RectangularPoseArea> areas;

    private Area(RectangularPoseArea... areas) {
      this.areas = Arrays.asList(areas);
    }
  }

  private static Pose2d[] rectangleToPolygon(RectangularPoseArea r) {
    // Extract min/max coordinates
    double minX = r.getMinX();
    double maxX = r.getMaxX();
    double minY = r.getMinY();
    double maxY = r.getMaxY();

    // Build the corners in clockwise order (last point closes the loop)
    return new Pose2d[] {
      new Pose2d(minX, minY, new Rotation2d()),
      new Pose2d(maxX, minY, new Rotation2d()),
      new Pose2d(maxX, maxY, new Rotation2d()),
      new Pose2d(minX, maxY, new Rotation2d()),
      new Pose2d(minX, minY, new Rotation2d())
    };
  }

  public List<Area> areas = List.of(Area.values());

  public RobotState(Subsystems subsystems) {
    this.drivetrain = subsystems.drivetrain();
    this.intakeWrist = subsystems.intakeWrist();
  }

  public Mode getMode() {
    Mode currentMode = Mode.NOTHING;

    if ("AUTO".equals(GameState.determineGameState().toString())) {
      currentMode = Mode.AUTO;
    } else {

    }

    return currentMode;
  }

  public Optional<Area> getArea() {
    return Arrays.stream(Area.values())
        .filter(
            a ->
                a.areas != null
                    && a.areas.stream()
                        .anyMatch(r -> r != null && r.contains(drivetrain.getEstimatedPose())))
        .findFirst();
  }

  public static DoubleSupplier getSpeedLimit() {
    return () -> {
      boolean isLimited = false;
      if (isLimited) {
        return 0.5;
      } else if (Controlboard.driveController.getLeftTriggerAxis()
          > Controlboard.TRIGGER_DEADBAND) {
        return 0.5;
      } else {
        return 1.0;
      }
    };
  }

  public void logArea() {
    int index = 0;
    for (Area area : Area.values()) {
      for (RectangularPoseArea rect : area.areas) {
        Logger.log("Field/Zones/" + area.name() + "_" + index++, rectangleToPolygon(rect));
      }
    }

    Logger.log("RobotState/Area Is Present", getArea().isPresent());
    if (getArea().isPresent()) {
      Logger.log("RobotState/Area", getArea().get().name());
    }
  }
}
