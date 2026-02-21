package frc2026.tars.subsystems.shooter.hood;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;

public class Hood extends TalonFXSubsystem {
  public static double angle;

  public Hood(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  public enum HoodGoal implements TalonFXSubsystemGoal {
    ZERO(Rotation2d.fromDegrees(0.0)),
    TOPOSE(Rotation2d.fromDegrees(angle));

    public Rotation2d angleRot;
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

  private double startTime = 0.0;

  public Command zero() {

    return new SequentialCommandGroup(
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        applyVoltageCommand(() -> -1.0)
            .withDeadline(
                new WaitUntilCommand(
                    () ->
                        ((Timer.getFPGATimestamp() - startTime) > 0.5)
                            && master.getSupplyCurrent().getValueAsDouble() > 1.0)),
        new InstantCommand(() -> resetPosition(0)));
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
