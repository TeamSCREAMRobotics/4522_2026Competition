package frc2026.tars;

import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.gameutil.GameState;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.Logger;
import com.teamscreamrobotics.zones.RectangularPoseArea;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc2026.tars.RobotContainer.Subsystems;
import frc2026.tars.controlboard.Controlboard;
import frc2026.tars.controlboard.Dashboard;
import frc2026.tars.subsystems.drivetrain.Drivetrain;
import frc2026.tars.subsystems.intake.IntakeWrist;
import frc2026.tars.subsystems.leds.LED;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;

public class RobotState {
  private final Drivetrain drivetrain;

  @SuppressWarnings("unused")
  private final IntakeWrist intakeWrist;

  @Getter @Setter private Rotation2d drivetrainTarget = Rotation2d.kZero;

  private final LED led;

  public enum Mode {
    AUTO,
    TELEOP,
    SHOOTINGTOHUB,
    FERRYING,
    CLIMBING,
    NOTHING
  }

  @SuppressWarnings("unchecked")
  public enum Area {
    ALLIANCEZONE(
        () -> AllianceFlipUtil.get(FieldConstants.BLUEALLIANCE, FieldConstants.REDALLIANCE)),

    OTHER_ALLIANCEZONE_DEPOT(
        () ->
            AllianceFlipUtil.get(
                FieldConstants.REDALLIANCE_DEPOT, FieldConstants.BLUEALLIANCE_DEPOT)),

    OTHER_ALLIANCEZONE_OUTPOST(
        () ->
            AllianceFlipUtil.get(
                FieldConstants.REDALLIANCE_OUTPOST, FieldConstants.BLUEALLIANCE_OUTPOST)),

    BUMPS(
        () -> FieldConstants.LeftBump.leftBump,
        () -> FieldConstants.LeftBump.oppLeftBump,
        () -> FieldConstants.RightBump.rightBump,
        () -> FieldConstants.RightBump.oppRightBump),

    TRENCHES(
        () -> FieldConstants.LeftTrench.leftTrench,
        () -> FieldConstants.LeftTrench.oppLeftTrench,
        () -> FieldConstants.RightTrench.rightTrench,
        () -> FieldConstants.RightTrench.oppRightTrench),

    DEPOT_SIDE_NEUTRALZONE(
        () -> AllianceFlipUtil.get(FieldConstants.UPPER_NEUTRAL, FieldConstants.LOWER_NEUTRAL)),

    OUTPOST_SIDE_NEUTRALZONE(
        () -> AllianceFlipUtil.get(FieldConstants.LOWER_NEUTRAL, FieldConstants.UPPER_NEUTRAL));

    private final Supplier<RectangularPoseArea>[] suppliers;
    private RectangularPoseArea[] resolved;

    Area(Supplier<RectangularPoseArea>... suppliers) {
      this.suppliers = suppliers;
    }

    void resolve() {

      resolved = new RectangularPoseArea[suppliers.length];

      for (int i = 0; i < suppliers.length; i++) {
        resolved[i] = suppliers[i].get();
      }
    }

    boolean contains(Pose2d pose) {

      if (resolved == null) return false;

      double x = pose.getX();
      double y = pose.getY();

      for (RectangularPoseArea area : resolved) {

        if (area != null
            && x >= area.getMinX()
            && x <= area.getMaxX()
            && y >= area.getMinY()
            && y <= area.getMaxY()) {

          return true;
        }
      }

      return false;
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

  public static final Area[] AREA_VALUES = Area.values();

  public RobotState(Subsystems subsystems) {
    this.drivetrain = subsystems.drivetrain();
    this.intakeWrist = subsystems.intakeWrist();
    this.led = subsystems.led();
  }

  public Mode getMode() {
    Mode currentMode = Mode.NOTHING;

    if ("AUTO".equals(GameState.determineGameState().toString())) {
      currentMode = Mode.AUTO;
    } else {

    }

    return currentMode;
  }

  private final boolean hopperElevated = false;

  @SuppressWarnings("unused")
  public Supplier<Translation2d> getInputTranslation() {
    Translation2d translation = Controlboard.getTranslation();
    double x = translation.getY();
    double y = translation.getX();
    Pose2d pose = drivetrain.getEstimatedPose();

    if (getArea() == Area.TRENCHES && hopperElevated) {
      if (pose.getX() < FieldConstants.LinesVertical.hubCenter) {
        y = Math.min(0, y); // Remove negative
      } else if (pose.getX() > FieldConstants.LinesVertical.hubCenter
          && pose.getX() < FieldConstants.LinesVertical.center) {
        y = Math.max(0, y); // Remove positive
      } else if (pose.getX() < FieldConstants.LinesVertical.oppHubCenter
          && pose.getX() > FieldConstants.LinesVertical.center) {
        y = Math.min(0, y);
      } else if (pose.getX() > FieldConstants.LinesVertical.oppHubCenter) {
        y = Math.max(0, y);
      }
    }

    final double finalX = x;
    final double finalY = y;
    return () -> new Translation2d(finalY, finalX);
  }

  private DriverStation.Alliance lastAlliance = null;

  private void resolveAreasIfNeeded() {

    var allianceOpt = DriverStation.getAlliance();

    if (allianceOpt.isEmpty()) return;

    var alliance = allianceOpt.get();

    if (alliance != lastAlliance) {

      lastAlliance = alliance;

      for (Area area : AREA_VALUES) {
        area.resolve();
      }
    }
  }

  public Area getArea() {

    resolveAreasIfNeeded();

    Pose2d pose = drivetrain.getEstimatedPose();

    for (Area area : AREA_VALUES) {

      if (area.contains(pose)) {
        return area;
      }
    }

    return null;
  }

  public static DoubleSupplier getSpeedLimit() {
    return () -> {
      boolean isLimited = false;
      if (isLimited) {
        return 0.5;
      } else if (Controlboard.driveController.leftStick().getAsBoolean()) {
        return 0.7;
      } else if (Controlboard.shoot().getAsBoolean()) {
        return 0.25;
      } else {
        return 1.0;
      }
    };
  }

  public boolean atTargetAngle() {
    return drivetrain.getWithinAngleThreshold(drivetrainTarget, Rotation2d.fromDegrees(7.5))
        || Dashboard.disableWaitUntilAim.get();
  }

  public void flashLEDS() {
    GameState gameStateUtil = new GameState();
    GameState.States hub = gameStateUtil.getActiveHub();
    GameState.States state = GameState.determineGameState();

    boolean flashing =
        state == GameState.States.SHIFTONEFLASHING
            || state == GameState.States.SHIFTTWOFLASHING
            || state == GameState.States.SHIFTTHREEFLASHING
            || state == GameState.States.SHIFTFOURFLASHING
            || state == GameState.States.ENDGAMEFLASHING;

    boolean normal =
        state == GameState.States.SHIFTONE
            || state == GameState.States.SHIFTTWO
            || state == GameState.States.SHIFTTHREE
            || state == GameState.States.SHIFTFOUR
            || state == GameState.States.ENDGAME;

    edu.wpi.first.wpilibj.util.Color hubColor;

    if (hub == GameState.States.RED) {
      hubColor = edu.wpi.first.wpilibj.util.Color.kGreen;
    } else {
      hubColor = edu.wpi.first.wpilibj.util.Color.kRed;
    }

    if (flashing) {
      led.strobe(hubColor, 0.2);
    } else if (normal) {
      led.breathe(hubColor, edu.wpi.first.wpilibj.util.Color.kBlack, 1.0);
    }
  }

  public void logArea() {
    for (Area area : AREA_VALUES) {

      if (area.resolved == null) continue;

      int index = 0;

      for (RectangularPoseArea rect : area.resolved) {
        Logger.log("Field/Zones/" + area.name() + "_" + index++, rectangleToPolygon(rect));
      }
    }
  }
}
