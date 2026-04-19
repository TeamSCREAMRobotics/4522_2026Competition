package frc2026.tars.subsystems.intake;

import com.teamscreamrobotics.dashboard.Ligament;
import com.teamscreamrobotics.dashboard.Mechanism;
import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc2026.tars.constants.SimConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class IntakeWrist extends TalonFXSubsystem {

  private final Ligament intakeOne =
      new Ligament()
          .withStaticLength(Length.fromInches(9.58))
          .withDynamicAngle(() -> getAngle(), () -> Rotation2d.fromRotations(getSetpoint()));
  private final Ligament intakeTwo =
      new Ligament()
          .withStaticLength(Length.fromInches(8.5))
          .withDynamicAngle(
              () -> getAngle().unaryMinus().minus(Rotation2d.fromDegrees(90)),
              () ->
                  Rotation2d.fromRotations(getSetpoint())
                      .unaryMinus()
                      .minus(Rotation2d.fromDegrees(90)));
  public final Mechanism intakeMech =
      new Mechanism("Intake Mech", intakeOne, intakeTwo)
          .withStaticPosition(
              new Translation2d(
                  (SimConstants.MECH_WIDTH / 2.0) + Units.inchesToMeters(12.125),
                  Units.inchesToMeters(8)));

  public IntakeWrist(TalonFXSubsystemConfiguration config) {
    super(config, IntakeWristGoal.EXTENDED);
  }

  public enum IntakeWristGoal implements TalonFXSubsystemGoal {
    STOW(Rotation2d.fromDegrees(0.0)),
    AGITATE_HIGH(Rotation2d.fromDegrees(30.0)),
    AGITATE_MID(Rotation2d.fromDegrees(50.0)),
    AGITATE_LOW(Rotation2d.fromDegrees(80.0)),
    COMPRESS(Rotation2d.fromDegrees(19.0)),
    EXTENDED(Rotation2d.fromDegrees(108.3));

    public final DoubleSupplier position;
    public final Rotation2d angle;

    private IntakeWristGoal(Rotation2d angle) {
      this.angle = angle;
      this.position = () -> angle.getRotations();
    }

    @Override
    public ControlType controlType() {
      return ControlType.MOTION_MAGIC_POSITION;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
    }

    public DoubleSupplier target() {
      return position;
    }
  }

  @Override
  public synchronized Command applyGoalCommand(TalonFXSubsystemGoal goal) {
    return super.applyGoalCommand(goal).beforeStarting(() -> super.goal = goal);
  }

  public Command moveToAngleCommand(Rotation2d targetAngle) {
    return run(
        () -> {
          setSetpointMotionMagicPosition(targetAngle.getRotations());
        });
  }

  public void moveToAngle(Rotation2d targetAngle) {
    setSetpointMotionMagicPosition(targetAngle.getRotations());
  }

  public Command compress(BooleanSupplier atTarget) {

    /* return new SequentialCommandGroup(
        new WaitUntilCommand(atTarget),
        new WaitCommand(2.0),
        run(
            () -> {
              setMotionMagicConfigsUnchecked(IntakeConstants.SLOW_MOTION_MAGIC_CONSTANTS);
              applyGoal(IntakeWristGoal.COMPRESS);
            }))
    .finallyDo(
        () -> setMotionMagicConfigsUnchecked(IntakeConstants.FAST_MOTION_MAGIC_CONSTANTS))
    .onlyIf(() -> shooterState.get() == ShooterState.SHOOTING); */
    return new SequentialCommandGroup(
        new WaitUntilCommand(atTarget),
        new WaitCommand(1.0),
        instantApplyGoalCommand(IntakeWristGoal.AGITATE_LOW),
        new WaitCommand(0.4),
        instantApplyGoalCommand(IntakeWristGoal.EXTENDED),
        new WaitCommand(0.4),
        instantApplyGoalCommand(IntakeWristGoal.AGITATE_MID),
        new WaitCommand(0.4),
        instantApplyGoalCommand(IntakeWristGoal.EXTENDED),
        new WaitCommand(0.4),
        Commands.repeatingSequence(
            instantApplyGoalCommand(IntakeWristGoal.AGITATE_HIGH),
            new WaitCommand(0.4),
            instantApplyGoalCommand(IntakeWristGoal.EXTENDED),
            new WaitCommand(0.4)));
  }

  public Command instantApplyGoalCommand(IntakeWristGoal goal) {
    return new InstantCommand(() -> applyGoal(goal), this);
  }

  @Getter public TalonFXSubsystemGoal goal = getGoal();

  @Override
  public void periodic() {
    super.periodic();

    Logger.log(logPrefix + "Angle", this.getAngle().getDegrees());
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
}
