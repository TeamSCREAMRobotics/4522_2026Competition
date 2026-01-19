package frc2026.robot.subsystems.shooter.flywheel;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

public class Flywheel extends TalonFXSubsystem {
  public Flywheel(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  public enum FlywheelGoal implements TalonFXSubsystemGoal {
    IDLE(() -> 1, ControlType.VELOCITY),
    SHOOTING(() -> 7, ControlType.VELOCITY);

    public final DoubleSupplier velocity;
    public final ControlType controlType;

    private FlywheelGoal(DoubleSupplier vel, ControlType controlType) {
      this.velocity = vel;
      this.controlType = controlType;
    }

    @Override
    public DoubleSupplier target() {
      return velocity;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0;
    }
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
