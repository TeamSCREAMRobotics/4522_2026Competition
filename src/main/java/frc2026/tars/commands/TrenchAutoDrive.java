package frc2026.tars.commands;

import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.BLinePathSequence;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FlippingUtil.FieldSymmetry;
import frc.robot.lib.BLine.FollowPath;
import frc2026.tars.RobotState.Area;
import java.util.function.Supplier;

public class TrenchAutoDrive {

  private final BLinePathSequence rightAllianceToNeutral;
  private final BLinePathSequence leftAllianceToNeutral;
  private final BLinePathSequence rightNeutralToAlliance;
  private final BLinePathSequence leftNeutralToAlliance;

  public TrenchAutoDrive(FollowPath.Builder pathBuilder) {

    rightAllianceToNeutral =
        new BLinePathSequence(pathBuilder, FieldSymmetry.kRotational, "AllianceToNeutral")
            .withName("Right Alliance To Neutral");
    leftAllianceToNeutral = rightAllianceToNeutral.mirror().withName("Left Alliance To Neutral");

    rightNeutralToAlliance =
        new BLinePathSequence(pathBuilder, FieldSymmetry.kRotational, "NeutralToAlliance")
            .withName("Right Neutral To Alliance");
    leftNeutralToAlliance = rightNeutralToAlliance.mirror().withName("Left Neutral To Alliance");
  }

  private boolean onLeftSide(Supplier<Pose2d> pose) {
    return AllianceFlipUtil.get(
        pose.get().getY() >= FieldConstants.fieldWidth / 2,
        pose.get().getY() <= FieldConstants.fieldWidth / 2);
  }

  private boolean inNeutralZone(Supplier<Area> area) {
    return area.get() == Area.DEPOT_SIDE_NEUTRALZONE || area.get() == Area.OUTPOST_SIDE_NEUTRALZONE;
  }

  public BLinePathSequence getCurrentTrenchPath(Supplier<Pose2d> pose, Supplier<Area> area) {
    if (inNeutralZone(area)) {
      return (onLeftSide(pose) ? leftNeutralToAlliance : rightNeutralToAlliance);
    } else {
      return (onLeftSide(pose) ? leftAllianceToNeutral : rightAllianceToNeutral);
    }
  }

  public Command driveThroughTrench(Supplier<Pose2d> pose, Supplier<Area> area) {
    return getCurrentTrenchPath(pose, area).getNext();
  }
}
