package frc2026.tars.subsystems.vision;

import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.util.GeomUtil;
import com.teamscreamrobotics.util.Logger;
import com.teamscreamrobotics.vision.LimelightHelpers;
import com.teamscreamrobotics.vision.LimelightHelpers.PoseEstimate;
import com.teamscreamrobotics.vision.LimelightVision.Limelight;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc2026.tars.Robot;
import frc2026.tars.subsystems.drivetrain.Drivetrain;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionManager {

  public static class Limelights {
    public static final Limelight turret =
        new Limelight(
            "limelight-turret",
            new Pose3d(
                0.0,
                Units.inchesToMeters(8.0),
                Units.inchesToMeters(18.5),
                new Rotation3d(0.0, Units.degreesToRadians(28.1), 0.0)));
    // public static final Limelight intake =
    //     new Limelight(
    //         "limelight-intake",
    //         new Pose3d(
    //             0.0,
    //             Units.inchesToMeters(10.5),
    //             Units.inchesToMeters(16.0),
    //             new Rotation3d(0, Units.degreesToRadians(-20.0), Units.degreesToRadians(35))));
    public static final Limelight swerveLeft =
        new Limelight(
            "limelight-left",
            new Pose3d(
                0.0,
                0.0,
                Units.inchesToMeters(8.0),
                new Rotation3d(0.0, Units.degreesToRadians(28.1), Math.PI)));
    public static final Limelight swerveRight =
        new Limelight(
            "limelight-right",
            new Pose3d(
                0.0,
                0.0,
                Units.inchesToMeters(8.0),
                new Rotation3d(0, Units.degreesToRadians(28.1), Math.PI)));
  }

  private PhotonCamera swerveLeft;
  private PhotonCamera swerveRight;
  private PhotonCamera intake;
  private PhotonCamera turretCam;
  private PhotonCamera[] cameras;
  private PhotonCameraSim swerveLeftSim;
  private PhotonCameraSim swerveRightSim;
  // private PhotonCameraSim intakeSim;
  private PhotonCameraSim turretSim;
  private PhotonCameraSim[] simCameras;
  private VisionSystemSim visionSim;

  private enum VisionType {
    REJECTED_INVALID,
    REJECTED_AMBIGUITY,
    REJECTED_MOVEMENT,
    MT,
    MT2;
  }

  private final Drivetrain drivetrain;
  private final Limelight[] limelights =
      new Limelight[] {Limelights.turret, Limelights.swerveLeft, Limelights.swerveRight};

  public static boolean hasEnabled = false;

  public VisionManager(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    if (Robot.isSimulation()) {
      swerveLeft = new PhotonCamera("left");
      swerveRight = new PhotonCamera("right");
      intake = new PhotonCamera("intake");
      turretCam = new PhotonCamera("turret");
      cameras = new PhotonCamera[] {swerveLeft, swerveRight, intake, turretCam};

      visionSim = new VisionSystemSim("main");

      visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded));

      var cameraProps = new SimCameraProperties();
      cameraProps.setCalibration(
          VisionConstants.resolutionWidth,
          VisionConstants.resolutionHeight,
          Rotation2d.fromDegrees(91.145));
      cameraProps.setCalibError(0.35, 0.10);
      cameraProps.setFPS(15.0);
      cameraProps.setAvgLatencyMs(10);
      cameraProps.setLatencyStdDevMs(3);

      swerveLeftSim = new PhotonCameraSim(swerveLeft, cameraProps);
      swerveRightSim = new PhotonCameraSim(swerveRight, cameraProps);
      // intakeSim = new PhotonCameraSim(intake, cameraProps);
      turretSim = new PhotonCameraSim(turretCam, cameraProps);

      simCameras = new PhotonCameraSim[] {swerveLeftSim, swerveRightSim, turretSim};

      visionSim.addCamera(
          swerveLeftSim, GeomUtil.pose3dToTransform3d(Limelights.swerveLeft.relativePosition()));
      visionSim.addCamera(
          swerveRightSim, GeomUtil.pose3dToTransform3d(Limelights.swerveRight.relativePosition()));
      visionSim.addCamera(
          turretSim, GeomUtil.pose3dToTransform3d(Limelights.swerveRight.relativePosition()));

      for (PhotonCameraSim camera : simCameras) {
        camera.enableRawStream(true);
        camera.enableProcessedStream(true);
        camera.enableDrawWireframe(false);
      }
    }
  }

  private void addGlobalPoseEstimate(Limelight limelight) {
    LimelightHelpers.SetRobotOrientation(
        limelight.name(),
        drivetrain.getHeading().getDegrees(),
        drivetrain.getYawRate().getDegrees(),
        0,
        0,
        0,
        0);
    PoseEstimate mt2Estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name());

    boolean shouldUseMt2 = hasEnabled && !rejectEstimate(mt2Estimate, limelight);

    if (shouldUseMt2) {
      double stdFactor = Math.pow(mt2Estimate.avgTagDist, 2.75) / (mt2Estimate.tagCount * 0.5);
      double xyStds = VisionConstants.xyStdBaseline * stdFactor * VisionConstants.xyMt2StdFactor;
      double thetaStds = VisionConstants.thetaStdBaseline * stdFactor;
      drivetrain.addVisionMeasurement(
          mt2Estimate.pose,
          mt2Estimate.timestampSeconds,
          VecBuilder.fill(xyStds, xyStds, 999999999999.0),
          true);

      Logger.log("Vision/" + limelight.name() + "/VisionType", VisionType.MT2);
      Logger.log("Vision/" + limelight.name() + "/PoseEstimate", mt2Estimate.pose, 1.5);
      Logger.log("Vision/" + limelight.name() + "/XyStds", xyStds);
      Logger.log("Vision/" + limelight.name() + "/ThetaStds", thetaStds);
    } else {
      Logger.log("Vision/" + limelight.name() + "/PoseEstimate", Pose2d.kZero);
      Logger.log("Vision/" + limelight.name() + "/XyStds", 0.0);
      Logger.log("Vision/" + limelight.name() + "/ThetaStds", 0.0);
    }
  }

  private boolean rejectEstimate(PoseEstimate estimate, Limelight limelight) {
    if (estimate == null
        || estimate.tagCount == 0
        || !FieldConstants.fieldArea.contains(estimate.pose)) {
      Logger.log("Vision/" + limelight.name() + "/VisionType", VisionType.REJECTED_INVALID);
      return true;
    } else if ((estimate.tagCount == 1 && estimate.rawFiducials[0].ambiguity > 0.3)) {
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

  public void periodic() {
    for (Limelight ll : limelights) {
      addGlobalPoseEstimate(ll);
    }

    if (Robot.isSimulation() && visionSim != null) {
      visionSim.update(drivetrain.getEstimatedPose());
      for (int i = 0; i < limelights.length; i++) {
        for (PhotonPipelineResult result : cameras[i].getAllUnreadResults()) {
          writeToTable(
              result,
              NetworkTableInstance.getDefault().getTable(limelights[i].name()),
              GeomUtil.pose3dToTransform3d(limelights[i].relativePosition()).inverse());
        }
      }
    }
  }

  private void writeToTable(
      PhotonPipelineResult result, NetworkTable table, Transform3d cameraToRobot) {
    double latencyMs = (Timer.getFPGATimestamp() - result.getTimestampSeconds()) * 1000.0;
    if (result.getMultiTagResult().isPresent()) {
      MultiTargetPNPResult multiTagResult = result.getMultiTagResult().get();
      Transform3d best = multiTagResult.estimatedPose.best.plus(cameraToRobot);
      Pose2d fieldToCamera =
          new Pose2d(best.getTranslation().toTranslation2d(), best.getRotation().toRotation2d());

      int targetCount = result.targets.size();
      List<Double> pose_data = new ArrayList<>(11);
      List<Double> rawFiducial_data = new ArrayList<>(targetCount * 7);

      pose_data.addAll(
          Arrays.asList(
              best.getX(),
              best.getY(),
              best.getZ(),
              0.0, // roll
              0.0, // pitch
              fieldToCamera.getRotation().getDegrees(),
              latencyMs,
              (double) multiTagResult.fiducialIDsUsed.size(),
              0.0, // tag span
              calculateAverageTagDistance(result), // avg tag dist
              result.getBestTarget().getArea()));

      for (PhotonTrackedTarget target : result.targets) {
        rawFiducial_data.add((double) target.getFiducialId());
        rawFiducial_data.add(target.getYaw());
        rawFiducial_data.add(target.getPitch());
        rawFiducial_data.add(target.getArea()); // ta
        rawFiducial_data.add(
            target.getBestCameraToTarget().getTranslation().getNorm()); // distToCamera
        rawFiducial_data.add(
            target
                .getBestCameraToTarget()
                .plus(cameraToRobot)
                .getTranslation()
                .getNorm()); // distToRobot
        rawFiducial_data.add(target.getPoseAmbiguity()); // ambiguity
      }

      double[] poseArray = pose_data.stream().mapToDouble(Double::doubleValue).toArray();
      double[] rawFiducialArray =
          rawFiducial_data.stream().mapToDouble(Double::doubleValue).toArray();
      table.getEntry("rawfiducials").setDoubleArray(rawFiducialArray);
      table.getEntry("botpose_wpiblue").setDoubleArray(poseArray);
      table.getEntry("botpose_orb_wpiblue").setDoubleArray(poseArray);
    }

    table.getEntry("tv").setInteger(result.hasTargets() ? 1 : 0);
    table
        .getEntry("cl")
        .setDouble((Timer.getFPGATimestamp() - result.getTimestampSeconds()) * 1000.0);
  }

  private double calculateAverageTagDistance(PhotonPipelineResult result) {
    double distance = 0;
    for (PhotonTrackedTarget target : result.targets) {
      distance += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    return distance / result.targets.size();
  }
}
