package frc2026.tars.subsystems.climber;

import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import com.teamscreamrobotics.math.Conversions;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;

public class Climber extends TalonFXSubsystem {

  public Climber(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  public enum ClimberGoal implements TalonFXSubsystemGoal {
    IDLE(Length.fromInches(0)),
    CLIMBING(Length.fromInches(33)); // Placeholder value, needs to be tuned

    public DoubleSupplier targetRotations;
    public Length height;

    private ClimberGoal(Length height) {
      this.height = height;
      this.targetRotations =
          () ->
              Conversions.linearDistanceToRotations(height, ClimberConstants.PULLEY_CIRCUMFERENCE);
    }

    @Override
    public DoubleSupplier target() {
      return targetRotations;
    }

    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0;
    }
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
                            && master.getSupplyCurrent().getValueAsDouble() > 1.8)),
        new InstantCommand(() -> resetPosition(0)));
  }
}
