package frc2026.tars.subsystems.shooter.feeder;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;

public class FeederConstants {
    public static final TalonFXSubsystemConfiguration FEEDER_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    FEEDER_CONFIG.name = "Feeder";

    FEEDER_CONFIG.codeEnabled = true;
    FEEDER_CONFIG.logTelemetry = false;
    FEEDER_CONFIG.debugMode = false;

    FEEDER_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(4), InvertedValue.Clockwise_Positive);
    FEEDER_CONFIG.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(5), InvertedValue.CounterClockwise_Positive)
        };

    FEEDER_CONFIG.neutralMode = NeutralModeValue.Coast;

    FEEDER_CONFIG.enableSupplyCurrentLimit = true;
    FEEDER_CONFIG.supplyCurrentLimit = 20;
  }
}
