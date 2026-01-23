// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2026.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2026.robot.subsystems.drivetrain.Drivetrain;
import frc2026.robot.subsystems.drivetrain.generated.TunerConstants;

public class RobotContainer {

  Drivetrain drivetrain = TunerConstants.drivetrain;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
