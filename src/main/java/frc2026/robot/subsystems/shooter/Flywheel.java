// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2026.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANrange;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

public class Flywheel extends TalonFXSubsystem {
  private static final CANrange beam = new CANrange(0);

  /** Creates a new Shooter. */
  public Flywheel(TalonFXSubsystemConfiguration config) {
    super(config);

    beam.getDistance().setUpdateFrequency(500.0);
  }

  public enum FlywheelGoal implements TalonFXSubsystemGoal {
    STOP(() -> 0.0, ControlType.VELOCITY),
    GO(() -> 1.1, ControlType.VELOCITY);

    public final DoubleSupplier velocity;
    public final ControlType controlType;

    private FlywheelGoal(DoubleSupplier velocity, ControlType controlType) {
      this.velocity = velocity;
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
      return velocity;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
