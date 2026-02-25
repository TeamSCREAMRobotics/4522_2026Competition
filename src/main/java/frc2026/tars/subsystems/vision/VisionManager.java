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
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc2026.tars.Robot;
import frc2026.tars.subsystems.drivetrain.Drivetrain;
import frc2026.tars.subsystems.shooter.turret.Turret;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
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
                Units.inchesToMeters(7.608245),
                Units.inchesToMeters(0.062500),
                Units.inchesToMeters(20.130854),
                new Rotation3d(0.0, Units.degreesToRadians(-28.1), 0.0)));
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
                Units.inchesToMeters(-12.095201),
                Units.inchesToMeters(6.856019),
                Units.inchesToMeters(8.597005),
                new Rotation3d(
                    0.0, Units.degreesToRadians(24.832735), Units.degreesToRadians(135.47249))));
    public static final Limelight swerveRight =
        new Limelight(
            "limelight-right",
            new Pose3d(
                Units.inchesToMeters(-12.094917),
                Units.inchesToMeters(-6.855182),
                Units.inchesToMeters(8.597005),
                new Rotation3d(
                    0.0, Units.degreesToRadians(24.832735), Units.degreesToRadians(-135.47249))));
  }

  private static class VisionUpdate {
    public final Pose2d pose;
    public final double timestamp;
    public final double xyStd;
    public final double thetaStd;
    public final String name;

    public VisionUpdate(Pose2d pose, double timestamp, double xyStd, double thetaStd, String name) {
      this.pose = pose;
      this.timestamp = timestamp;
      this.xyStd = xyStd;
      this.thetaStd = thetaStd;
      this.name = name;
    }
  }

  private void backgroundVisionUpdate() {

    processLimelight(Limelights.swerveLeft);
    processLimelight(Limelights.swerveRight);
    // processTurret(); // add later if desired
  }

  private PhotonCamera swerveLeft;
  private PhotonCamera swerveRight;
  // private PhotonCamera intake;
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

  private void processLimelight(Limelight limelight) {

    LimelightHelpers.SetRobotOrientation(
        limelight.name(),
        drivetrain.getHeading().getDegrees(),
        drivetrain.getYawRate().getDegrees(),
        0,
        0,
        0,
        0);

    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name());

    if (rejectEstimate(estimate, limelight)) {
      return;
    }

    double stdFactor = Math.pow(estimate.avgTagDist, 2.75) / (estimate.tagCount * 0.5);

    double xyStds = VisionConstants.xyStdBaseline * stdFactor * VisionConstants.xyMt2StdFactor;

    double thetaStds = VisionConstants.thetaStdBaseline * stdFactor;

    pendingUpdates.add(
        new VisionUpdate(
            estimate.pose, estimate.timestampSeconds, xyStds, thetaStds, limelight.name()));
  }

  private final Drivetrain drivetrain;
  private final Turret turret;
  private final Limelight[] limelights =
      new Limelight[] {Limelights.swerveLeft, Limelights.swerveRight, Limelights.turret};

  private final Notifier visionThread;
  private final ConcurrentLinkedQueue<VisionUpdate> pendingUpdates = new ConcurrentLinkedQueue<>();

  public static final Transform3d turretToCameraFixed =
      new Transform3d(
          Units.inchesToMeters(7.608245),
          Units.inchesToMeters(0.062500),
          Units.inchesToMeters(4.630854),
          Rotation3d.kZero);
  public static final Transform3d robotToTurretFixed =
      new Transform3d(
          Units.inchesToMeters(-5.765591),
          Units.inchesToMeters(0.000143),
          Units.inchesToMeters(15.5),
          Rotation3d.kZero);

  public VisionManager(Drivetrain drivetrain, Turret turret) {
    this.drivetrain = drivetrain;
    this.turret = turret;

    visionThread = new Notifier(this::backgroundVisionUpdate);

    if (Robot.isSimulation()) {
      swerveLeft = new PhotonCamera("limelight-left");
      swerveRight = new PhotonCamera("limelight-right");
      // intake = new PhotonCamera("intake");
      turretCam = new PhotonCamera("limelight-turret");
      cameras = new PhotonCamera[] {swerveLeft, swerveRight, turretCam};

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
          turretSim, GeomUtil.pose3dToTransform3d(Limelights.turret.relativePosition()));

      for (PhotonCameraSim camera : simCameras) {
        camera.enableRawStream(true);
        camera.enableProcessedStream(true);
        camera.enableDrawWireframe(false);
      }
    }
  }

  private void addTurretEstimate() {
    Limelight turretCam = Limelights.turret;

    LimelightHelpers.SetRobotOrientation(
        turretCam.name(),
        drivetrain.getHeading().plus(turret.getAngle()).getDegrees(),
        drivetrain.getYawRate().plus(Rotation2d.fromRotations(turret.getVelocity())).getDegrees(),
        0,
        0,
        0,
        0);
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(turretCam.name());

    if (rejectEstimate(estimate, turretCam)) {
      return;
    }

    Pose3d fieldToTurret = new Pose3d(estimate.pose);

    Pose3d fieldToRobot = fieldToTurret.transformBy(robotToTurretFixed.inverse());

    estimate.pose = fieldToRobot.toPose2d();

    addPoseEstimate(estimate, turretCam);
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

    PoseEstimate poseEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.name());
    addPoseEstimate(poseEstimate, limelight);
  }

  public void stop(){
    visionThread.stop();
  }

  public void start(){
    visionThread.startPeriodic(0.02);
    //  runs every 20 ms.
}



  private void addPoseEstimate(PoseEstimate estimate, Limelight limelight) {
    boolean shouldUseMt2 = !rejectEstimate(estimate, limelight);

    if (shouldUseMt2) {
      double stdFactor = Math.pow(estimate.avgTagDist, 2.75) / (estimate.tagCount * 0.5);
      double xyStds = VisionConstants.xyStdBaseline * stdFactor * VisionConstants.xyMt2StdFactor;
      double thetaStds = VisionConstants.thetaStdBaseline * stdFactor;
      drivetrain.addVisionMeasurement(
          estimate.pose,
          estimate.timestampSeconds,
          VecBuilder.fill(xyStds, xyStds, 999999999999.0),
          true);

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

  public void periodic() {
    VisionUpdate update;

    if ((update = pendingUpdates.poll()) != null) {

      drivetrain.addVisionMeasurement(
          update.pose,
          update.timestamp,
          VecBuilder.fill(update.xyStd, update.xyStd, 999999999999.0),
          true);

      Logger.log("Vision/" + update.name + "/VisionType", VisionType.MT2);
      Logger.log("Vision/" + update.name + "/PoseEstimate", update.pose);
      Logger.log("Vision/" + update.name + "/XyStds", update.xyStd);
      Logger.log("Vision/" + update.name + "/ThetaStds", update.thetaStd);
      System.out.println("Vision thread is running.");
    }
    // addStaticEstimate(Limelights.swerveLeft);
    // addStaticEstimate(Limelights.swerveRight);
    // addTurretEstimate();

    if (Robot.isSimulation() && visionSim != null) {
      visionSim.update(drivetrain.getEstimatedPose());
      visionSim.adjustCamera(turretSim, getRobotToTurretCamera());
      for (int i = 0; i < limelights.length; i++) {
        for (PhotonPipelineResult result : cameras[i].getAllUnreadResults()) {
          writeToTable(
              result,
              NetworkTableInstance.getDefault().getTable(limelights[i].name()),
              visionSim.getRobotToCamera(simCameras[i]).get().inverse());
        }
      }
      Logger.log("Turret Camera Pose", visionSim.getCameraPose(turretSim).get());
    }
  }

  private Transform3d getRobotToTurretCamera() {
    return robotToTurretFixed
        .plus(new Transform3d(GeomUtil.rotationToTransform(turret.getAngle())))
        .plus(
            new Transform3d(
                turretToCameraFixed.getTranslation(),
                new Rotation3d(0, Limelights.turret.relativePosition().getRotation().getY(), 0)));
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
