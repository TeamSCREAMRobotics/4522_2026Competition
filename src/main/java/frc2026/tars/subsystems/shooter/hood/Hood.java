package frc2026.tars.subsystems.shooter.hood;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Hood extends TalonFXSubsystem {
  public Hood(TalonFXSubsystemConfiguration config) {
    super(config);
    resetPosition(0);
  }

  // public Command moveToAngleCommand(Rotation2d targetAngle) {
  //   return run(
  //       () -> {
  //         setSetpointMotionMagicPosition(targetAngle.getRotations());
  //       });
  // }

  public void moveToAngleCommand(Rotation2d targetAngle) {
    setSetpointMotionMagicPosition(targetAngle.getRotations());
  }

  private double startTime = 0.0;

  public Command zero() {

    return new SequentialCommandGroup(
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        applyVoltageCommand(() -> .5)
            .withDeadline(
                new WaitUntilCommand(
                    () ->
                        ((Timer.getFPGATimestamp() - startTime) > 0.5)
                            && master.getSupplyCurrent().getValueAsDouble() > .01)),
        new InstantCommand(() -> startTime = 0),
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        applyVoltageCommand(() -> -.5)
            .withDeadline(
                new WaitUntilCommand(
                    () ->
                        ((Timer.getFPGATimestamp() - startTime) > 0.5)
                            && master.getSupplyCurrent().getValueAsDouble() > .01)),
        new InstantCommand(() -> resetPosition(0)));
  }

  @Override
  public void periodic() {
    super.periodic();

    Logger.log(logPrefix + "Angle", getAngle().getDegrees());
  }
}
