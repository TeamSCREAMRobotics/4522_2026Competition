package frc2026.robot.subsystems.climber;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

public class Climber extends TalonFXSubsystem {
  public Climber(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  public enum ClimberGoal implements TalonFXSubsystemGoal {
    STOW(() -> 0.0, ControlType.MOTION_MAGIC_POSITION);

    public final DoubleSupplier height;
    public final ControlType controlType;

    private ClimberGoal(DoubleSupplier height, ControlType controlType) {
      this.height = height;
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
      return height;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
