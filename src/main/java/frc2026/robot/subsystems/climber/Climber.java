package frc2026.robot.subsystems.climber;

import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import com.teamscreamrobotics.math.Conversions;
import com.teamscreamrobotics.math.ScreamMath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  public synchronized Command applyGoalCommand(TalonFXSubsystemGoal goal) {
    return super.applyGoalCommand(goal).beforeStarting(() -> super.goal = goal);
  }

  public Command applyUntilAtGoalCommand(ClimberGoal goal) {
    return super.applyGoalCommand(goal)
        .until(() -> atGoal())
        .beforeStarting(() -> super.goal = goal);
  }

  public double getHeightPercent() {
    return (MathUtil.clamp(
                getMeasuredHeight().getInches(), 0, ClimberConstants.MAX_HEIGHT.getInches())
            / ClimberConstants.MAX_HEIGHT.getInches())
        * 100.0;
  }

  public Length getMeasuredHeight() {
    return Length.fromRotations(getPosition(), ClimberConstants.PULLEY_CIRCUMFERENCE);
  }

  public Length getSetpointHeight() {
    return Length.fromRotations(getSetpoint(), ClimberConstants.PULLEY_CIRCUMFERENCE);
  }

  @SuppressWarnings("unused")
  private double rotationsToHeightInches(double rotations) {
    return rotations * (2.256 * Math.PI);
  }

  public static double heightToRotations(Length height) {
    return Conversions.linearDistanceToRotations(height, ClimberConstants.PULLEY_CIRCUMFERENCE);
  }

  public double getDriveScalar() {
    double clampedHeight =
        MathUtil.clamp(
            getMeasuredHeight().getInches(), 0.0, ClimberConstants.MAX_HEIGHT.getInches());

    return ScreamMath.mapRange(
        clampedHeight, 0.0, ClimberConstants.MAX_HEIGHT.getInches(), 3.9, 1.0);
  }

  public double getAccelScalar() {
    double clampedHeight =
        MathUtil.clamp(
            getMeasuredHeight().getInches(), 0.0, ClimberConstants.MAX_HEIGHT.getInches());

    return ScreamMath.mapRange(
        clampedHeight, 0.0, ClimberConstants.MAX_HEIGHT.getInches(), 4.3, 3.0);
  }

  private double startTime = 0.0;

  public Command rezero() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> startTime = Timer.getFPGATimestamp()),
        applyVoltageCommand(() -> -2.0)
            .withDeadline(
                new WaitUntilCommand(
                    () ->
                        ((Timer.getFPGATimestamp() - startTime) > 0.5)
                            && master.getSupplyCurrent().getValueAsDouble() > 17.0)),
        new InstantCommand(() -> resetPosition(0.0)));
  }

  public Command quickRezero() {
    return applyVoltageCommand(() -> -2.0)
        .withTimeout(0.25)
        .finallyDo(
            (interrupted) -> {
              if (!interrupted) {
                resetPosition(0.0);
              }
            })
        // .andThen(new InstantCommand(() -> resetPosition(0.0)))
        .withName("Rezero");
  }

  public void resetSimController() {
    simController.reset(getPosition(), getVelocity());
  }

  public void setGoal(ClimberGoal goal) {
    super.goal = goal;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
