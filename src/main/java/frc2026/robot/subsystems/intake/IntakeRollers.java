package frc2026.robot.subsystems.intake;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

public class IntakeRollers extends TalonFXSubsystem {
  public IntakeRollers(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  public enum IntakeRollersGoal implements TalonFXSubsystemGoal {
    STOW(() -> 0.0, ControlType.VOLTAGE);

    public final DoubleSupplier voltage;
    public final ControlType controlType;

    private IntakeRollersGoal(DoubleSupplier voltage, ControlType controlType) {
      this.voltage = voltage;
      this.controlType = controlType;
    }

    @Override
    public ControlType controlType() {
      return controlType;
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
  public void periodic() {}
}
