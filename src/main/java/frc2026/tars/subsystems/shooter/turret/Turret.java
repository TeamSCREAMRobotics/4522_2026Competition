package frc2026.tars.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.teamscreamrobotics.dashboard.Ligament;
import com.teamscreamrobotics.dashboard.Mechanism;
import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.math.ScreamMath;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc2026.tars.Robot;
import frc2026.tars.constants.Constants;
import frc2026.tars.constants.Constants.RobotType;
import frc2026.tars.constants.SimConstants;
import frc2026.tars.subsystems.vision.VisionManager;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Turret extends TalonFXSubsystem {
  private final Ligament turretTwo =
      new Ligament()
          .withStaticLength(Length.fromInches(5.0))
          .withDynamicAngle(() -> Rotation2d.fromDegrees(getAngle().getDegrees()));

  private final Ligament turretOne =
      new Ligament()
          .withStaticLength(Length.fromInches(5.0))
          .withStaticAngle(Rotation2d.fromDegrees(90));

  public final Mechanism turretMech =
      new Mechanism("Turret Mech", turretOne, turretTwo)
          .withStaticPosition(
              new Translation2d(
                  (SimConstants.MECH_WIDTH / 2.0) + Units.inchesToMeters(12.125),
                  Units.inchesToMeters(15)));

  public final Mechanism2d robotTest = new Mechanism2d(1, 1);

  public final MechanismRoot2d robotRoot = robotTest.getRoot(getName(), 0.5, 0.5);

  public final MechanismLigament2d turret = new MechanismLigament2d("turret", 0.4, 0.0);
  private final SysIdRoutine routine;

  /** Creates a new Pivot Subsystem. */
  public Turret(TalonFXSubsystemConfiguration config) {
    super(config);

    // Initialize motor controller
    robotRoot.append(turret);

    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(7), // Use dynamic voltage of 7 V
                null, // Use default timeout (10 s)
                // Log state with SignalLogger class
                state ->
                    SignalLogger.writeString(
                        "SysIdTurret_State", state.toString())), // Default config is fine for most
            new SysIdRoutine.Mechanism(
                volts -> master.setVoltage(volts.in(Volts)), // Apply voltage to the motor
                null,
                this));

    if (Constants.robot == RobotType.COMPBOT) {
      resetPosition(0);
    }
  }

  /** Update simulation and telemetry. */
  @Override
  public void periodic() {
    super.periodic();
    if (Robot.isSimulation()) {
      turret.setAngle(getAngle().getDegrees());

      SmartDashboard.putData("turret mech", robotTest);
    }

    Logger.log(logPrefix + "Motor Angle", getAngle().getDegrees());
  }

  // Does the actual check to ensure that angle is within bounds
  // and will not damage any parts of the turret.
  private boolean isWithinLimits(double angle) {
    return angle >= TurretConstants.MIN_ROT_DEG && angle <= TurretConstants.MAX_ROT_DEG;
  }

  private Rotation2d getSafeTargetAngle(Rotation2d requestedAngle) {
    Rotation2d current = getAngle();
    double currentDegrees = current.getDegrees();

    // shortest circular difference
    double delta =
        Units.radiansToDegrees(MathUtil.angleModulus(requestedAngle.minus(current).getRadians()));

    // two possible path
    double pathCW = delta > 0 ? delta - 360 : delta;
    double pathCCW = delta < 0 ? delta + 360 : delta;

    double endCW = currentDegrees + pathCW;
    double endCCW = currentDegrees + pathCCW;

    boolean cwValid = isWithinLimits(endCW);
    boolean ccwValid = isWithinLimits(endCCW);

    double chosenDelta;

    if (cwValid && ccwValid) {
      chosenDelta = Math.abs(pathCW) < Math.abs(pathCCW) ? pathCW : pathCCW;
    } else if (cwValid) {
      chosenDelta = pathCW;
    } else if (ccwValid) {
      chosenDelta = pathCCW;
    } else {
      // No legal path — clamp to nearest limit
      return Rotation2d.fromDegrees(
          MathUtil.clamp(
              requestedAngle.getDegrees(),
              TurretConstants.MIN_ROT_DEG,
              TurretConstants.MAX_ROT_DEG));
    }

    return Rotation2d.fromDegrees(
        MathUtil.clamp(
            chosenDelta + currentDegrees,
            TurretConstants.MIN_ROT_DEG,
            TurretConstants.MAX_ROT_DEG));
  }

  /**
   * Creates a command to move the pivot to a specific angle with a profile.
   *
   * @param angle The target angle in degrees
   * @return A command that moves the pivot to the specified angle
   */
  // This is the robot relative verion of this command.
  public Command moveToAngleCommandRR(Supplier<Rotation2d> angle) {
    return run(
        () -> {
          Rotation2d safeTarget = getSafeTargetAngle(angle.get());
          setSetpointMotionMagicPosition(safeTarget.getRotations());
        });
  }

  /**
   * Creates a command to move the pivot to a specific angle with a profile.
   *
   * @param angle The target angle in degrees
   * @return A command that moves the pivot to the specified angle
   */
  // This is the robot relative verion of this command.
  public Command moveToAngleCommandRR(Rotation2d angle) {
    return moveToAngleCommandRR(() -> angle);
  }

  /**
   * Creates a command to move the turret to a specific angle field relitive.
   *
   * @param angle The target angle in degrees
   * @return A command that moves turret to a specific angle field relitive
   */
  public Command moveToAngleCommandFR(
      Supplier<Rotation2d> angle, Supplier<Rotation2d> robotHeading) {
    return moveToAngleCommandRR(() -> (angle.get().minus(robotHeading.get())));
  }

  /**
   * Creates a command to move the turret to a specific angle field relitive.
   *
   * @param angle The target angle in degrees
   * @return A command that moves turret to a specific angle field relitive
   */
  public Command moveToAngleCommandFR(Rotation2d angle, Supplier<Rotation2d> robotHeading) {
    return moveToAngleCommandFR(() -> angle, robotHeading);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public Command setZero() {
    return runOnce(() -> resetPosition(0));
  }

  public Command pointAtFieldPosition(
      Supplier<Translation2d> targetPosition, Supplier<Pose2d> robotPose) {
    return moveToAngleCommandFR(
        () ->
            ScreamMath.calculateAngleToPoint(
                targetPosition.get(), robotPose.get().getTranslation()),
        () -> robotPose.get().getRotation());
  }

  public Command pointAtFieldPosition(Translation2d targetPosition, Supplier<Pose2d> robotPose) {
    return pointAtFieldPosition(() -> targetPosition, robotPose);
  }

  /**
   * Creates a command to point the turret at the hub center. Uses the hub center position from
   * FieldConstants.
   *
   * @return A command that points the turret at the hub center
   */
  public Command pointAtHubCenter(Supplier<Pose2d> robotPose) {
    return pointAtFieldPosition(
            () ->
                AllianceFlipUtil.get(
                    FieldConstants.Hub.innerCenterPoint.toTranslation2d(),
                    FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()),
            robotPose)
        .withName("PointAtHubCenter");
  }

  public Command aimOnTheFlyCommand(
      Supplier<Translation2d> targetPosition,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> robotSpeed,
      DoubleSupplier timeOfFlight) {
    return run(() ->
            aimOnTheFly(
                targetPosition.get(),
                robotPose.get(),
                robotSpeed.get(),
                timeOfFlight.getAsDouble()))
        .withName("AimOnTheFly");
  }

  public void aimOnTheFly(
      Translation2d targetPosition,
      Pose2d robotPose,
      ChassisSpeeds robotSpeed,
      double timeOfFlight) {
    double predictionTime = TurretConstants.LATENCY + timeOfFlight;

    Translation2d fieldVelocity =
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond)
            .rotateBy(robotPose.getRotation());

    Translation2d futureRobotTranslation =
        robotPose.getTranslation().plus(fieldVelocity.times(predictionTime));

    Rotation2d futureRotation =
        robotPose
            .getRotation()
            .plus(Rotation2d.fromRadians(robotSpeed.omegaRadiansPerSecond * predictionTime));

    Translation2d futureTurretTranslation =
        futureRobotTranslation.plus(
            VisionManager.robotToTurretFixed
                .getTranslation()
                .toTranslation2d()
                .rotateBy(futureRotation));

    Translation2d futureTargetVector = targetPosition.minus(futureTurretTranslation);

    Rotation2d turretRobotRelativeAngle = futureTargetVector.getAngle().minus(futureRotation);

    Rotation2d safeTarget = getSafeTargetAngle(turretRobotRelativeAngle);

    setSetpointMotionMagicPosition(safeTarget.getRotations());

    Logger.log(logPrefix + "Safe Target", safeTarget.getDegrees());

    Logger.log(
        "AimOnTheFly/FutureTurretTranslation", new Pose2d(futureTurretTranslation, futureRotation));
  }
}
