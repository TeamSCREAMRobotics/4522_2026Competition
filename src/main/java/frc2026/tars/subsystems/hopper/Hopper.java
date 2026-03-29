package frc2026.tars.subsystems.hopper;

import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import com.teamscreamrobotics.math.Conversions;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Hopper extends TalonFXSubsystem {

  public Hopper(TalonFXSubsystemConfiguration config) {
    super(config, HopperGoal.IDLE);
  }

  public enum HopperGoal implements TalonFXSubsystemGoal {
    IDLE(Length.fromInches(0.0)),
    EXTENDED(Length.fromInches(3.0));

    public DoubleSupplier targetRotations;
    public Length height;

    private HopperGoal(Length height) {
      this.height = height;
      this.targetRotations =
          () -> Conversions.linearDistanceToRotations(height, Length.fromInches(1.6));
    }

    @Override
    public DoubleSupplier target() {
      return targetRotations;
    }

    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0;
    }
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
                            && master.getSupplyCurrent().getValueAsDouble() > 1.8)),
        new InstantCommand(() -> resetPosition(0)));
  }

  public Command extend(BooleanSupplier atTarget) {
    return Commands.run(
            () -> {
              if (atTarget.getAsBoolean() == true) {
                setMotionMagicConfigsUnchecked(HopperConstants.SLOW_MOTION_MAGIC_CONSTANTS);
                applyGoal(HopperGoal.EXTENDED);
              }
            })
        .finallyDo(
            () -> setMotionMagicConfigsUnchecked(HopperConstants.FAST_MOTION_MAGIC_CONSTANTS));
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
