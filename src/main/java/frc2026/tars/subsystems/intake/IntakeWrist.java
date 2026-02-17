// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2026.tars.subsystems.intake;

import com.teamscreamrobotics.dashboard.Ligament;
import com.teamscreamrobotics.dashboard.Mechanism;
import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc2026.tars.constants.SimConstants;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
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
    super(config, IntakeWristGoal.STOW);
  }

  public enum IntakeWristGoal implements TalonFXSubsystemGoal {
    STOW(() -> Units.degreesToRotations(111.6736)),
    EXTENDED(() -> Units.degreesToRotations(45));

    public final DoubleSupplier position;
    ;

    private IntakeWristGoal(DoubleSupplier position) {
      this.position = position;
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
}
