package frc2026.tars.subsystems.shooter.indexer;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

public class Spindexer extends TalonFXSubsystem {

  public Spindexer(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  public enum SpindexerGoal implements TalonFXSubsystemGoal {
    STOP(() -> 0.0),
    RUN(() -> 4.5);

    public final DoubleSupplier voltage;

    private SpindexerGoal(DoubleSupplier voltage) {
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
}
