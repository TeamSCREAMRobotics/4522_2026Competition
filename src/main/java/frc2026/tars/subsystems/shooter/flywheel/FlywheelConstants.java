package frc2026.tars.subsystems.shooter.flywheel;

import com.ctre.phoenix6.signals.InvertedValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;

public class FlywheelConstants {
  public static final double FLYWHEEL_REDUCTION = 1.0;

  public static final TalonFXSubsystemConfiguration FLYWHEEL_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    FLYWHEEL_CONFIG.name = "Flywheel";

    FLYWHEEL_CONFIG.codeEnabled = false;
    FLYWHEEL_CONFIG.logTelemetry = false;
    FLYWHEEL_CONFIG.debugMode = false;

    FLYWHEEL_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(10), InvertedValue.Clockwise_Positive);
    FLYWHEEL_CONFIG.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(11), InvertedValue.CounterClockwise_Positive)
        };

    FLYWHEEL_CONFIG.enableSupplyCurrentLimit = true;
    FLYWHEEL_CONFIG.supplyCurrentLimit = 20;
    FLYWHEEL_CONFIG.sensorToMechRatio = FLYWHEEL_REDUCTION;
  }
}
