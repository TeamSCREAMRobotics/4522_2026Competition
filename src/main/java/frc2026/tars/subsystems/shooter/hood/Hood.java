package frc2026.tars.subsystems.shooter.hood;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;

public class Hood extends TalonFXSubsystem {
  public static DoubleSupplier angle;

  public Hood(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  public Command moveToAngleCommand(Rotation2d targetAngle) {
    return run(
        () -> {
          double targetRotations = 0;
          targetRotations = targetAngle.getRotations();
          setSetpointMotionMagicPosition(targetRotations);
        });
  }

  private double startTime = 0.0;

  public Command zero() {

    return new SequentialCommandGroup(
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        applyVoltageCommand(() -> -1.0)
            .withDeadline(
                new WaitUntilCommand(
                    () ->
                        ((Timer.getFPGATimestamp() - startTime) > 0.5)
                            && master.getSupplyCurrent().getValueAsDouble() > .0006)),
        new InstantCommand(() -> resetPosition(0)));
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
