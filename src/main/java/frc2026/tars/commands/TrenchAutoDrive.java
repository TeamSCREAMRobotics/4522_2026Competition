package frc2026.tars.commands;

import com.teamscreamrobotics.util.BLinePathSequence;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.lib.BLine.FlippingUtil.FieldSymmetry;
import frc.robot.lib.BLine.FollowPath;
import frc2026.tars.subsystems.drivetrain.Drivetrain;

public class TrenchAutoDrive {
  private final Drivetrain drivetrain;
  private final FollowPath.Builder pathBuilder;

  private final BLinePathSequence rightAllianceToNeutral;
  private final BLinePathSequence leftAllianceToNeutral;
  private final BLinePathSequence rightNeutralToAlliance;
  private final BLinePathSequence leftNeutralToAlliance;

  public TrenchAutoDrive(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    pathBuilder =
        new FollowPath.Builder(
                drivetrain,
                drivetrain::getEstimatedPose,
                () -> drivetrain.getState().Speeds,
                (speeds) ->
                    drivetrain.setControl(drivetrain.getHelper().getApplyRobotSpeeds(speeds)),
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(3.0, 0.0, 0.0),
                new PIDController(2.0, 0.0, 0.0))
            .withDefaultShouldFlip();

    rightAllianceToNeutral = new BLinePathSequence(pathBuilder, FieldSymmetry.kRotational, "");
    leftAllianceToNeutral = rightAllianceToNeutral.mirror();

    rightNeutralToAlliance = new BLinePathSequence(pathBuilder, FieldSymmetry.kRotational, "");
    leftNeutralToAlliance = rightNeutralToAlliance.mirror();
  }

  // private boolean onLeftSide(Supplier<Pose2d> pose) {
  //     if (pose.get().getY() AllianceFlipUtil.get(">", "<") FieldConstants.fieldWidth / 2) {

  //     }
  // }

  // public BLinePathSequence getCurrentTrenchPath() {
  //     if
  // }
}
