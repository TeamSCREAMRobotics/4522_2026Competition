// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2026.robot.subsystems.intake;

import com.teamscreamrobotics.dashboard.Ligament;
import com.teamscreamrobotics.dashboard.Mechanism;
import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc2026.robot.constants.SimConstants;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class IntakeWrist extends TalonFXSubsystem {

  private final Ligament intakeOne =
      new Ligament()
          .withStaticLength(Length.fromInches(5.0))
          .withDynamicAngle(() -> getAngle(), () -> Rotation2d.fromRotations(getSetpoint()));
  private final Ligament intakeTwo =
      new Ligament()
          .withStaticLength(Length.fromInches(5.0))
          .withStaticAngle(Rotation2d.fromDegrees(45.0));
  public final Mechanism intakeMech =
      new Mechanism("Intake Mech", intakeOne, intakeTwo)
          .withStaticPosition(new Translation2d(SimConstants.MECH_WIDTH / 2.0, 0.0));

  public IntakeWrist(TalonFXSubsystemConfiguration config) {
    super(config, IntakeWristGoal.EXTENDED);
  }

  public enum IntakeWristGoal implements TalonFXSubsystemGoal {
    STOW(() -> 0.0, ControlType.MOTION_MAGIC_POSITION),
    EXTENDED(() -> 2.5, ControlType.MOTION_MAGIC_POSITION);

    public final DoubleSupplier position;
    public final ControlType controlType;

    private IntakeWristGoal(DoubleSupplier position, ControlType controlType) {
      this.position = position;
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

    public DoubleSupplier target() {
      return position;
    }
  }

  @Override
  public synchronized Command applyGoalCommand(TalonFXSubsystemGoal goal) {
    return super.applyGoalCommand(goal).beforeStarting(() -> super.goal = goal);
  }
}
