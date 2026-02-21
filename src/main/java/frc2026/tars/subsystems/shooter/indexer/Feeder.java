package frc2026.tars.subsystems.shooter.indexer;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;

public class Feeder extends TalonFXSubsystem {
  public Feeder(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  public enum FeederGoal implements TalonFXSubsystemGoal {
    STOP(() -> 0.0),
    RUN(() -> 4.5);

    public final DoubleSupplier voltage;

    private FeederGoal(DoubleSupplier voltage) {
      this.voltage = voltage;
    }

    @Override
    public ControlType controlType() {
      return ControlType.VOLTAGE;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
    }

    @Override
    public DoubleSupplier target() {
      return voltage;
    }
  }

  private double startTime = 0.0;

  public Command unClog() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        applyVoltageCommand(() -> -2.0)
            .withDeadline(
                new WaitUntilCommand(() -> ((Timer.getFPGATimestamp() - startTime) > 1.5))));
  }
}
