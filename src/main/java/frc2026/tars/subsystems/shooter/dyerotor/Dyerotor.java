package frc2026.tars.subsystems.shooter.dyerotor;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;

public class Dyerotor extends TalonFXSubsystem {

  public Dyerotor(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  private double counter = 2.25d;

  public void runDyerotor() {

    counter++;
    setVoltage(Math.min(6.7, counter * counter));
  }

  public void stopDyerotor() {
    setVoltage(0.0);
  }
}
