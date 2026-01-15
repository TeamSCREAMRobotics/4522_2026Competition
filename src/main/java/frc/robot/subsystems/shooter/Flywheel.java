// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.CANrange;
import com.teamscreamrobotics.drivers.TalonFXSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends TalonFXSubsystem {
  private static final CANrange beam = new CANrange(0);
  /** Creates a new Shooter. */
  public Flywheel(TalonFXSubsystemConfiguration config) {
    super(config);

    beam.getDistance().setUpdateFrequency(500.0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
