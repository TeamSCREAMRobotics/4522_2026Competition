// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2026.tars;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.teamscreamrobotics.util.Logger;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc2026.tars.controlboard.Dashboard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();

    Dashboard.initialize();

    Logger.setOptions(
        new DogLogOptions()
            .withCaptureDs(false)
            .withCaptureNt(false)
            .withLogExtras(true)
            .withNtPublish(true)
            .withUseLogThread(false)
            .withLogEntryQueueCapacity(2000));
    Logger.setEnabled(true);

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

    Threads.setCurrentThreadPriority(true, 10);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_robotContainer.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    m_robotContainer.getVisionManager().start();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.getVisionManager().start();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    m_robotContainer.getVisionManager().stop();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.getVisionManager().start();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    m_robotContainer.getVisionManager().stop();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
