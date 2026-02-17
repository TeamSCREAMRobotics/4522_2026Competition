package frc2026.tars.subsystems.shooter.flywheel;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc2026.tars.subsystems.shooter.hood.Hood.HoodGoal;
import java.util.function.DoubleSupplier;

public class Flywheel extends TalonFXSubsystem {
  public static double shootVel;

  public Flywheel(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  public enum FlywheelGoal implements TalonFXSubsystemGoal {
    IDLE(() -> 1),
    SHOOTING(() -> shootVel);

    public DoubleSupplier velocity;

    private FlywheelGoal(DoubleSupplier vel) {
      this.velocity = vel;
    }

    @Override
    public DoubleSupplier target() {
      return velocity;
    }

    @Override
    public ControlType controlType() {
      return ControlType.VELOCITY;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0;
    }
  }

  public Command applyUntilAtGoalCommand(HoodGoal goal) {
    return super.applyGoalCommand(goal).until(() -> atGoal());
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
