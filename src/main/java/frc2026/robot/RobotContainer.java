// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2026.robot;

import com.teamscreamrobotics.dashboard.MechanismVisualizer;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc2026.robot.constants.SimConstants;
import frc2026.robot.controlboard.Controlboard;
import frc2026.robot.subsystems.intake.IntakeConstants;
import frc2026.robot.subsystems.intake.IntakeWrist;
import frc2026.robot.subsystems.intake.IntakeWrist.IntakeWristGoal;
import frc2026.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc2026.robot.subsystems.swerve.TunerConstants;
import lombok.Getter;

public class RobotContainer {

  public record Subsystems(CommandSwerveDrivetrain drivetrain, IntakeWrist intakeWrist) {}

  private final CommandXboxController joystick = new CommandXboxController(0);

  private final IntakeWrist intakeWrist = new IntakeWrist(IntakeConstants.WRIST_CONFIG);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  @Getter private final Subsystems subsystems = new Subsystems(drivetrain, intakeWrist);

  @Getter private final RobotState robotState = new RobotState(subsystems);

  private final MechanismVisualizer mechVisualizer =
      new MechanismVisualizer(
          SimConstants.MEASURED_MECHANISM,
          SimConstants.SETPOINT_MECHANISM,
          RobotContainer::telemeterizeMechanisms,
          intakeWrist.intakeMech);

  public RobotContainer() {
    configureBindings();
    SmartDashboard.putNumber("test", 1);

    mechVisualizer.setEnabled(true);

    drivetrain.setDefaultCommand(
        drivetrain
            .applyRequest(
                () ->
                    Controlboard.getFieldCentric().getAsBoolean()
                        ? drivetrain
                            .getHelper()
                            .getHeadingCorrectedFieldCentric(
                                Controlboard.getTranslation()
                                    .get()
                                    .times(RobotState.getSpeedLimit().getAsDouble()),
                                Controlboard.getRotation().getAsDouble())
                        : drivetrain
                            .getHelper()
                            .getRobotCentric(
                                Controlboard.getTranslation()
                                    .get()
                                    .times(RobotState.getSpeedLimit().getAsDouble()),
                                Controlboard.getRotation().getAsDouble()))
            .beforeStarting(() -> drivetrain.getHelper().setLastAngle(drivetrain.getHeading()))
            .withName("Drivetrain: Default command"));
  }

  private void configureBindings() {
    joystick
        .a()
        .whileTrue(intakeWrist.applyGoalCommand(IntakeWristGoal.EXTENDED))
        .onFalse(intakeWrist.applyGoalCommand(IntakeWristGoal.STOW));
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static void telemeterizeMechanisms(Mechanism2d measured, Mechanism2d setpoint) {
    Logger.log("RobotState/Mechanisms/Measured", measured);
    Logger.log("RobotState/Mechanisms/Setpoint", setpoint);
  }

  public void logState() {
    robotState.logArea();
  }
}
