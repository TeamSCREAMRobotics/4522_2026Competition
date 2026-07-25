package frc2026.tars.subsystems.vision;

import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.util.Logger;
import com.teamscreamrobotics.vision.LimelightHelpers;
import com.teamscreamrobotics.vision.LimelightHelpers.PoseEstimate;
import com.teamscreamrobotics.vision.LimelightVision.Limelight;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc2026.tars.controlboard.Dashboard;
import frc2026.tars.subsystems.drivetrain.Drivetrain;

public class VisionManager {

  public static class Limelights {
    public static final Limelight shooter =
        new Limelight(
            "limelight-shooter",
            new Pose3d(
                Units.inchesToMeters(-13.066),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(13.925),
                new Rotation3d(180.0, Units.degreesToRadians(-25.0), 0.0)));

    public static final Limelight right =
        new Limelight(
            "limelight-right",
            new Pose3d(
                Units.inchesToMeters(0.220),
                Units.inchesToMeters(12.670),
                Units.inchesToMeters(18.984),
                new Rotation3d(0.0, Units.degreesToRadians(0.0), Units.degreesToRadians(90.0))));

    public static final Limelight left =
        new Limelight(
            "limelight-left",
            new Pose3d(
                Units.inchesToMeters(0.220),
                Units.inchesToMeters(-12.670),
                Units.inchesToMeters(18.285),
                new Rotation3d(180.0, Units.degreesToRadians(0.0), Units.degreesToRadians(-90.0))));
  }

  private enum VisionType {
    REJECTED_INVALID,
    REJECTED_AMBIGUITY,
    REJECTED_MOVEMENT,
    MT,
    MT2;
  }

  private final Drivetrain drivetrain;

  @SuppressWarnings("unused")
  public VisionManager(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    if (false) {
      return;
    }
  }

  private void addStaticEstimate(Limelight limelight) {
    LimelightHelpers.SetRobotOrientation(
        limelight.name(),
        drivetrain.getHeading().getDegrees(),
        drivetrain.getYawRate().getDegrees(),
        0,
        0,
        0,
        0);

    if (DriverStation.isDisabled()) {
      addPoseEstimate(
          LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.name()), limelight, true, false);
    } else {
      addPoseEstimate(
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name()),
          limelight,
          false,
          false);
    }
  }

  private void addPoseEstimate(
      PoseEstimate estimate, Limelight limelight, boolean mt1, boolean isTurret) {
    boolean shouldUseMt2 = !rejectEstimate(estimate, limelight);

    if (shouldUseMt2 && !Dashboard.disableAllVisionUpdates.get()) {
      double stdFactor = Math.pow(estimate.avgTagDist, 2.75) / (estimate.tagCount * 0.5);
      double xyStds =
          VisionConstants.xyStdBaseline
              * stdFactor
              * (mt1 ? 1.0 : VisionConstants.xyMt2StdFactor)
              * (isTurret ? VisionConstants.xyTurretFactor * estimate.avgTagDist : 1.0);
      double thetaStds =
          DriverStation.isDisabled() ? 0.5 : VisionConstants.thetaStdBaseline * stdFactor;
      drivetrain.addVisionMeasurement(
          estimate.pose,
          estimate.timestampSeconds,
          VecBuilder.fill(xyStds, xyStds, mt1 ? thetaStds : 999999999999.0),
          !mt1);

      Logger.log("Vision/" + limelight.name() + "/VisionType", VisionType.MT2);
      Logger.log("Vision/" + limelight.name() + "/PoseEstimate", estimate.pose);
      Logger.log("Vision/" + limelight.name() + "/XyStds", xyStds);
      Logger.log("Vision/" + limelight.name() + "/ThetaStds", thetaStds);
    } else {
      Logger.log("Vision/" + limelight.name() + "/PoseEstimate", Pose2d.kZero);
      Logger.log("Vision/" + limelight.name() + "/XyStds", 0.0);
      Logger.log("Vision/" + limelight.name() + "/ThetaStds", 0.0);
    }
  }

  @SuppressWarnings("unused")
  public void periodic() {
    addStaticEstimate(Limelights.right);
    addStaticEstimate(Limelights.shooter);
    addStaticEstimate(Limelights.left);
  }

  private boolean rejectEstimate(PoseEstimate estimate, Limelight limelight) {
    if (estimate == null
        || estimate.tagCount == 0
        || !FieldConstants.fieldArea.contains(estimate.pose)) {
      Logger.log("Vision/" + limelight.name() + "/VisionType", VisionType.REJECTED_INVALID);
      return true;
    } else if ((estimate.tagCount == 1 && estimate.rawFiducials[0].ambiguity > 0.3)
        && !Dashboard.disableAmbiguityRejection.get()) {
      Logger.log("Vision/" + limelight.name() + "/VisionType", VisionType.REJECTED_AMBIGUITY);
      return true;
    } else if ((Math.abs(drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble())
            > 540)
        || (drivetrain.getLinearVelocity().getNorm() > 3.5)) {
      Logger.log("Vision/" + limelight.name() + "/VisionType", VisionType.REJECTED_MOVEMENT);
      return true;
    } else {
      return false;
    }
  }
}