package frc2026.tars.subsystems.intake;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeRollers extends TalonFXSubsystem {

  public IntakeRollers(TalonFXSubsystemConfiguration config) {
    super(config, IntakeRollersGoal.STOP);
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public enum IntakeRollersGoal implements TalonFXSubsystemGoal {
    STOP(() -> 0.0),
    COMPRESS(() -> 7.7),
    INTAKE(() -> 10.0),
    AUTOINTAKE(() -> 12.0),
    OUTTAKE(() -> -6.0);

    public final DoubleSupplier voltage;

    private IntakeRollersGoal(DoubleSupplier voltage) {
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

  @Override
  public synchronized void setVoltage(double volts, double voltageFeedForward) {
    setMaster(voltageRequest.withOutput(volts + voltageFeedForward).withEnableFOC(false));
    if (shouldSimulate()) {
      simulationThread.setSimVoltage(() -> volts + voltageFeedForward);
    }
  }
}
