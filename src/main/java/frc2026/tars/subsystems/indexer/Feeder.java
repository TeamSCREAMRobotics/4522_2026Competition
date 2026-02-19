package frc2026.tars.subsystems.indexer;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
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
}
