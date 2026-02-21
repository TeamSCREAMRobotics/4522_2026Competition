package frc2026.tars.subsystems.shooter.flywheel;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;

import java.util.function.DoubleSupplier;

public class Flywheel extends TalonFXSubsystem {
  public static DoubleSupplier shootVel;

  public Flywheel(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
