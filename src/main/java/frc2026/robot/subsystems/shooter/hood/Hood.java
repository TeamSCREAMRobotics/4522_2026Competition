package frc2026.robot.subsystems.shooter.hood;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class Hood extends TalonFXSubsystem {
  private static double angle;

  public Hood(TalonFXSubsystemConfiguration config, double angle) {
    super(config);
    Hood.angle = angle;
  }

  public enum HoodGoal implements TalonFXSubsystemGoal {
    ZERO(Rotation2d.fromDegrees(0.0)),
    TOPOSE(Rotation2d.fromDegrees(angle));

    Rotation2d angleRot;
    DoubleSupplier target;

    HoodGoal(Rotation2d angleRot) {
      this.angleRot = angleRot;
      this.target = () -> angleRot.getRotations();
    }

    @Override
    public DoubleSupplier target() {
      return target;
    }

    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
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
