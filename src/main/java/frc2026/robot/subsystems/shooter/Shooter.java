package frc2026.robot.subsystems.shooter;

import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.physics.Trajectory;
import com.teamscreamrobotics.physics.Trajectory.GamePiece;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2026.robot.subsystems.shooter.flywheel.Flywheel;
import frc2026.robot.subsystems.shooter.flywheel.Flywheel.FlywheelGoal;
import frc2026.robot.subsystems.shooter.hood.Hood;
import frc2026.robot.subsystems.shooter.hood.Hood.HoodGoal;
import frc2026.robot.subsystems.shooter.hood.HoodConstants;

public class Shooter extends SubsystemBase {
  Hood hood;
  Flywheel flywheel;

  private static final InterpolatingDoubleTreeMap angleLookup = new InterpolatingDoubleTreeMap();
  private static final String logPrefix = "Shooter/";

  public Shooter(Hood hood, Flywheel flywheel) {
    this.hood = hood;
    this.flywheel = flywheel;

    initializeLookupTables();
  }

  private void initializeLookupTables() {
    // TODO: MUST be tuned with actual testing!
    // key: Distance (meters), value: Hood Angle (degrees)

    // angleLookup.put(1.0, 45.0);
    // angleLookup.put(2.0, 40.0);
    // angleLookup.put(3.0, 35.0);
    // angleLookup.put(4.0, 32.0);
    // angleLookup.put(4.5, 30.0);

    generatePhysicsBasedAngleLookup();
  }

  private void generatePhysicsBasedAngleLookup() {
    for (double distance = 1.0; distance <= 5.0; distance += 0.5) {
      double testVelocity = 12.0;

      Trajectory.configure()
          .setTargetDistance(distance)
          .setTargetHeight(FieldConstants.Hub.height)
          .setInitialHeight(ShooterConstants.HEIGHT)
          .setShotVelocity(testVelocity)
          .setGamePiece(GamePiece.FUEL);

      double optimalAngle = Trajectory.getOptimalAngle();

      if (optimalAngle > 0) {
        optimalAngle =
            Math.max(
                HoodConstants.HOOD_MIN_ANGLE, Math.min(HoodConstants.HOOD_MAX_ANGLE, optimalAngle));
        angleLookup.put(distance, optimalAngle);
      }
    }
  }

  public Command shootCommand(double velocity, Rotation2d angle) {
    return runEnd(
        () -> {
          Flywheel.shootVel = velocity;
          Hood.angle = angle.getDegrees();
          hood.applyGoalCommand(HoodGoal.TOPOSE);
          flywheel.applyGoalCommand(FlywheelGoal.SHOOTING);
        },
        () -> {
          hood.stop();
          flywheel.stop();
        });
  }









  

  public Command shootAtTargetCommand(Translation2d target) {
    return runEnd(
        () -> {
          double velocity = calculateFlywheelVelocity(target);
          Rotation2d angle = calculateHoodAngle(target);

          Flywheel.shootVel = velocity;
          Hood.angle = angle.getDegrees();
          hood.applyGoalCommand(HoodGoal.TOPOSE);
          flywheel.applyGoalCommand(FlywheelGoal.SHOOTING);
        },
        () -> {
          hood.stop();
          flywheel.stop();
        });
  }

  private double calculateHorizontalDistance(Translation2d target) {
    return target.getNorm();
  }

  public double calculateFlywheelVelocity(Translation2d target) {
    double horizontalDistance = calculateHorizontalDistance(target);

    double hoodAngle = angleLookup.get(horizontalDistance);

    hoodAngle =
        Math.max(HoodConstants.HOOD_MIN_ANGLE, Math.min(HoodConstants.HOOD_MAX_ANGLE, hoodAngle));

    Trajectory.configure()
        .setTargetDistance(horizontalDistance)
        .setTargetHeight(FieldConstants.Hub.height)
        .setInitialHeight(ShooterConstants.HEIGHT)
        .setShotAngle(hoodAngle)
        .setGamePiece(GamePiece.FUEL);

    double requiredVelocity = Trajectory.getRequiredVelocity();

    if (requiredVelocity < 0) {
      return calculateFallbackVelocity(horizontalDistance, hoodAngle);
    }

    return requiredVelocity;
  }

  public Rotation2d calculateHoodAngle(Translation2d target) {
    double horizontalDistance = calculateHorizontalDistance(target);
    double angleDegrees = angleLookup.get(horizontalDistance);

    angleDegrees =
        Math.max(
            HoodConstants.HOOD_MIN_ANGLE, Math.min(HoodConstants.HOOD_MAX_ANGLE, angleDegrees));

    return Rotation2d.fromDegrees(angleDegrees);
  }

  public double[] calculateOptimalShot(Translation2d target) {
    double horizontalDistance = calculateHorizontalDistance(target);

    double testVelocity = 12.0;

    Trajectory.configure()
        .setTargetDistance(horizontalDistance)
        .setTargetHeight(FieldConstants.Hub.height)
        .setInitialHeight(ShooterConstants.HEIGHT)
        .setShotVelocity(testVelocity)
        .setGamePiece(GamePiece.FUEL);

    double optimalAngle = Trajectory.getOptimalAngle();

    if (optimalAngle < 0) {
      optimalAngle = angleLookup.get(horizontalDistance);
    }

    optimalAngle =
        Math.max(
            HoodConstants.HOOD_MIN_ANGLE, Math.min(HoodConstants.HOOD_MAX_ANGLE, optimalAngle));

    Trajectory.configure()
        .setTargetDistance(horizontalDistance)
        .setTargetHeight(FieldConstants.Hub.height)
        .setInitialHeight(ShooterConstants.HEIGHT)
        .setShotAngle(optimalAngle)
        .setGamePiece(GamePiece.FUEL);

    double requiredVelocity = Trajectory.getRequiredVelocity();

    if (requiredVelocity < 0) {
      requiredVelocity = calculateFallbackVelocity(horizontalDistance, optimalAngle);
    }

    return new double[] {requiredVelocity, optimalAngle};
  }

  private double calculateFallbackVelocity(double horizontalDistance, double angleDegrees) {
    double angleRad = Math.toRadians(angleDegrees);
    double heightDiff = FieldConstants.Hub.height - ShooterConstants.HEIGHT;

    double cosAngle = Math.cos(angleRad);
    double tanAngle = Math.tan(angleRad);

    double denominator = 2 * cosAngle * cosAngle * (horizontalDistance * tanAngle - heightDiff);

    if (denominator <= 0) {
      return 15.0;
    }

    double velocitySquared =
        (ShooterConstants.GRAVITY * horizontalDistance * horizontalDistance) / denominator;
    return Math.sqrt(Math.abs(velocitySquared));
  }

  public void addHoodAngleDataPoint(double distance, double angleDegrees) {
    angleLookup.put(distance, angleDegrees);
    System.out.printf("Added hood angle data: %.1fm -> %.1f°%n", distance, angleDegrees);
  }

  public void printPhysicsPrediction(double distance, double angle) {
    Trajectory.configure()
        .setTargetDistance(distance)
        .setTargetHeight(FieldConstants.Hub.height)
        .setInitialHeight(ShooterConstants.HEIGHT)
        .setShotAngle(angle)
        .setGamePiece(GamePiece.FUEL);

    double velocity = Trajectory.getRequiredVelocity();
    double timeOfFlight = Trajectory.getTimeOfFlight();

    Logger.log(
        logPrefix + "PhysicsPrediction",
        String.format(
            "Distance: %.1fm, Angle: %.1f°, Velocity: %.2f m/s, Time: %.3f s",
            distance, angle, velocity, timeOfFlight));
  }

  public void generateTuningReport() {
    for (double distance = 1.0; distance <= 5.0; distance += 0.5) {
      double angle = angleLookup.get(distance);

      Trajectory.configure()
          .setTargetDistance(distance)
          .setTargetHeight(FieldConstants.Hub.height)
          .setInitialHeight(ShooterConstants.HEIGHT)
          .setShotAngle(angle)
          .setGamePiece(GamePiece.FUEL);

      double velocity = Trajectory.getRequiredVelocity();
      String notes = (velocity < 0) ? "NO SOLUTION" : "OK";

      Logger.log(logPrefix + "Distance", distance);
      Logger.log(logPrefix + "Angle", angle);
      Logger.log(logPrefix + "Velocity", velocity);
      Logger.log(logPrefix + "Notes", notes);
    }
  }

  public double velocityToRPM(
      double velocityMS, double wheelDiameterInches, double compressionInches) {
    double effectiveRadiusInches = (wheelDiameterInches - compressionInches) / 2.0;
    double effectiveRadiusMeters = effectiveRadiusInches * 0.0254;

    double circumference = 2 * Math.PI * effectiveRadiusMeters;
    double rotationsPerSecond = velocityMS / circumference;

    return rotationsPerSecond * 60.0;
  }

  public boolean isReadyToShoot() {
    return hood.atGoal() && flywheel.atGoal();
  }

  public double getEstimatedTimeOfFlight(Translation2d target) {
    double horizontalDistance = calculateHorizontalDistance(target);
    double angle = angleLookup.get(horizontalDistance);

    Trajectory.configure()
        .setTargetDistance(horizontalDistance)
        .setTargetHeight(FieldConstants.Hub.height)
        .setInitialHeight(ShooterConstants.HEIGHT)
        .setShotAngle(angle)
        .setGamePiece(GamePiece.FUEL);

    Trajectory.getRequiredVelocity();
    return Trajectory.getTimeOfFlight();
  }

  @Override
  public void periodic() {}
}
