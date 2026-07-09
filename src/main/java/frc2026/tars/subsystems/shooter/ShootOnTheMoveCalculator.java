package frc2026.tars.subsystems.shooter;

import com.teamscreamrobotics.math.Conversions;
import com.teamscreamrobotics.math.ScreamMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc2026.tars.subsystems.shooter.flywheel.FlywheelConstants;
import java.util.function.DoubleUnaryOperator;

public final class ShootOnTheMoveCalculator {
  private static final int SOLVE_ITERATIONS = 20;

  private ShootOnTheMoveCalculator() {}

  public record Solution(
      Translation2d correctedLaunchPosition,
      double correctedDistanceMeters,
      Rotation2d drivetrainHeading,
      double flywheelVelocityRps,
      double hoodAngleDegrees,
      double timeOfFlightSeconds,
      ChassisSpeeds launcherFieldVelocity) {}

  public static Solution calculate(
      Pose2d robotPose,
      Pose2d fieldToShooter,
      ChassisSpeeds fieldVelocity,
      Translation2d target,
      DoubleUnaryOperator flywheelVelocityForDistance,
      DoubleUnaryOperator hoodAngleForDistance,
      double phaseDelaySeconds,
      double linearDragConstant) {
    Translation2d launcherPosition = fieldToShooter.getTranslation();
    double launcherOffsetX = launcherPosition.getX() - robotPose.getX();
    double launcherOffsetY = launcherPosition.getY() - robotPose.getY();

    double launcherVx =
        fieldVelocity.vxMetersPerSecond - launcherOffsetY * fieldVelocity.omegaRadiansPerSecond;
    double launcherVy =
        fieldVelocity.vyMetersPerSecond + launcherOffsetX * fieldVelocity.omegaRadiansPerSecond;

    Translation2d phasedLauncherPosition =
        launcherPosition.plus(
            new Translation2d(launcherVx * phaseDelaySeconds, launcherVy * phaseDelaySeconds));

    Translation2d correctedLaunchPosition = phasedLauncherPosition;
    double correctedDistanceMeters = correctedLaunchPosition.getDistance(target);
    double timeOfFlightSeconds = 0.0;

    for (int i = 0; i < SOLVE_ITERATIONS; i++) {
      double flywheelVelocityRps =
          flywheelVelocityForDistance.applyAsDouble(correctedDistanceMeters);
      double hoodAngleDegrees = hoodAngleForDistance.applyAsDouble(correctedDistanceMeters);

      timeOfFlightSeconds =
          calculateTimeOfFlight(correctedDistanceMeters, flywheelVelocityRps, hoodAngleDegrees);
      double effectiveTimeOfFlight =
          applyLinearDragCompensation(timeOfFlightSeconds, linearDragConstant);

      correctedLaunchPosition =
          phasedLauncherPosition.plus(
              new Translation2d(
                  launcherVx * effectiveTimeOfFlight, launcherVy * effectiveTimeOfFlight));
      correctedDistanceMeters = correctedLaunchPosition.getDistance(target);
    }

    double flywheelVelocityRps = flywheelVelocityForDistance.applyAsDouble(correctedDistanceMeters);
    double hoodAngleDegrees = hoodAngleForDistance.applyAsDouble(correctedDistanceMeters);
    timeOfFlightSeconds =
        calculateTimeOfFlight(correctedDistanceMeters, flywheelVelocityRps, hoodAngleDegrees);

    return new Solution(
        correctedLaunchPosition,
        correctedDistanceMeters,
        ScreamMath.calculateAngleToPoint(correctedLaunchPosition, target).plus(Rotation2d.k180deg),
        flywheelVelocityRps,
        hoodAngleDegrees,
        timeOfFlightSeconds,
        new ChassisSpeeds(launcherVx, launcherVy, fieldVelocity.omegaRadiansPerSecond));
  }

  private static double applyLinearDragCompensation(
      double timeOfFlightSeconds, double linearDragConstant) {
    if (Math.abs(linearDragConstant) < 1e-6) {
      return timeOfFlightSeconds;
    }

    return (1.0 - Math.exp(-timeOfFlightSeconds * linearDragConstant)) / linearDragConstant;
  }

  private static double calculateTimeOfFlight(
      double distanceMeters, double flywheelVelocityRps, double hoodAngleDegrees) {
    double exitVelocity =
        Conversions.rpsToMPS(
                flywheelVelocityRps,
                FlywheelConstants.FLYWHEEL_CIRCUMFERENCE.getMeters(),
                FlywheelConstants.FLYWHEEL_REDUCTION)
            * 0.8;
    double exitAngle = 90.0 - hoodAngleDegrees;
    double horizontalVelocity = exitVelocity * Math.cos(Units.degreesToRadians(exitAngle));

    if (horizontalVelocity < 0.01) {
      return distanceMeters / 0.01;
    }

    return distanceMeters / horizontalVelocity;
  }
}
