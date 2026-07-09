package frc2026.tars;

import com.ctre.phoenix6.SignalLogger;
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
    Dashboard.initialize();
    m_robotContainer = new RobotContainer();

    Logger.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureNt(false)
            .withLogExtras(true)
            .withNtPublish(true)
            .withUseLogThread(false)
            .withLogEntryQueueCapacity(2000));
    Logger.setEnabled(true);

    SignalLogger.start();

    Threads.setCurrentThreadPriority(true, 10);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    Dashboard.periodic();
    m_robotContainer.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    Dashboard.autonInit();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
