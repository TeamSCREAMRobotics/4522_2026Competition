package frc2026.tars.subsystems.shooter.dyerotor;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class Dyerotor extends TalonFXSubsystem {

  public Timer runTimer = new Timer();

  public Dyerotor(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  public double sixPointSeven = sixPointSeven();

  public double sixPointSeven() {
    return 6.7;
  }

  public void runDyerotor() {
    runTimer.start();
    if (runTimer.get() >= .5) {
      setVoltage(5.3);
    } else {
      setVoltage(2.0);
    }
  }

  @Override
  public void stop() {
    runTimer.reset();
    super.stop();
  }
}
