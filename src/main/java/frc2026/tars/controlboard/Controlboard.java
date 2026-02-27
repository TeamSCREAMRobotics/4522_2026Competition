package frc2026.tars.controlboard;

import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.ScreamUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2026.tars.subsystems.drivetrain.DrivetrainConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Controlboard {
  public static final CommandXboxController driveController = new CommandXboxController(0);

  public static final double STICK_DEADBAND = 0.05;
  public static final double TRIGGER_DEADBAND = 0.1;
  public static final Rotation2d SNAP_TO_POLE_THRESHOLD = Rotation2d.fromDegrees(4.0);

  public static boolean fieldCentric = true;

  static {
    driveController.start().onTrue(Commands.runOnce(() -> fieldCentric = !fieldCentric));
  }

  public static double applyPower(double value, int power) {
    return Math.pow(value, power) * (power % 2 == 0 ? Math.signum(value) : 1);
  }

  public static Supplier<Translation2d> getRawTranslation() {
    return () -> new Translation2d(driveController.getLeftY(), driveController.getLeftX());
  }

  public static Supplier<Translation2d> getTranslation() {
    return () ->
        snapTranslationToPole(
                new Translation2d(
                        -applyPower(
                            MathUtil.applyDeadband(driveController.getLeftY(), STICK_DEADBAND), 2),
                        -applyPower(
                            MathUtil.applyDeadband(driveController.getLeftX(), STICK_DEADBAND), 2))
                    .times(DrivetrainConstants.maxSpeed))
            .times(AllianceFlipUtil.getDirectionCoefficient());
  }

  public static Translation2d snapTranslationToPole(Translation2d translation) {
    if (!translation.equals(Translation2d.kZero)) {
      for (int i = 0; i < 4; i++) {
        if (ScreamUtil.withinAngleThreshold(
            Rotation2d.kCCW_90deg.times(i), translation.getAngle(), SNAP_TO_POLE_THRESHOLD)) {
          return new Translation2d(translation.getNorm(), Rotation2d.kCCW_90deg.times(i));
        }
      }
      return translation;
    } else {
      return Translation2d.kZero;
    }
  }

  public static DoubleSupplier getRotation() {
    return () ->
        -applyPower(MathUtil.applyDeadband(driveController.getRightX(), STICK_DEADBAND), 3)
            * DrivetrainConstants.maxAngularSpeedRads;
  }

  public static BooleanSupplier getFieldCentric() {
    return () -> fieldCentric;
  }

  public static Trigger intake() {
    return driveController.leftTrigger(TRIGGER_DEADBAND);
  }

  public static Trigger intakeUp() {
    return driveController.leftBumper();
  }

  public static Trigger shoot() {
    return driveController.rightTrigger(TRIGGER_DEADBAND);
  }

  public static Trigger zeroIntake() {
    return new Trigger(() -> Dashboard.zeroIntake.get());
  }

  public static Trigger zeroClimber() {
    return new Trigger(() -> Dashboard.zeroClimber.get());
  }

  public static Trigger zeroHood() {
    return new Trigger(() -> Dashboard.zeroHood.get());
  }

  public static Trigger zeroTurret() {
    return new Trigger(() -> Dashboard.zeroTurret.get());
  }

  public static Trigger resetManuals() {
    return new Trigger(() -> Dashboard.resetManuals.get());
  }

  public static Trigger getManualMode() {
    return new Trigger(() -> Dashboard.manualMode.get());
  }

  public static Trigger hailMaryMode(){
    return new Trigger(() -> Dashboard.hailMaryMode.get());
  }

  public static Trigger resetFieldCentric() {
    return driveController.back();
  }

  public static Trigger makeThingWork() {
    return driveController.b();
  }

  public static Trigger moveIntakeWrist() {
    return driveController.leftBumper();
  }

  public static Trigger lockSwerve(){
    return driveController.povDown();
  }
}
