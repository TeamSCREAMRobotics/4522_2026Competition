// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2026.robot.subsystems.intake;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class IntakeWrist extends TalonFXSubsystem {

  public IntakeWrist(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  @Override
  public void periodic() {}

  public enum IntakeWristGoal implements TalonFXSubsystemGoal {
    STOW(() -> 0.0, ControlType.MOTION_MAGIC_POSITION);

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
}
