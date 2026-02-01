// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2026.robot.subsystems.shooter;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;

public class Hood extends TalonFXSubsystem {
  /** Creates a new Hood. */
  public Hood(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  public enum HoodGoal implements TalonFXSubsystemGoal {
    STOW(() -> Rotation2d.fromDegrees(0.0).getDegrees(), ControlType.POSITION);

    public final DoubleSupplier position;
    public final ControlType controlType;

    private HoodGoal(DoubleSupplier position, ControlType controlType) {
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

    @Override
    public DoubleSupplier target() {
      return position;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
