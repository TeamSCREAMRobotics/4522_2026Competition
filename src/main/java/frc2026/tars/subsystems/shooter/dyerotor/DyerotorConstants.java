package frc2026.tars.subsystems.shooter.dyerotor;

import com.ctre.phoenix6.signals.InvertedValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.*;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;

public class DyerotorConstants {
  public static final TalonFXSubsystemConfiguration DYEROTOR_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    DYEROTOR_CONFIG.name = "Spindexer";

    DYEROTOR_CONFIG.codeEnabled = true;
    DYEROTOR_CONFIG.logTelemetry = false;
    DYEROTOR_CONFIG.debugMode = false;

    DYEROTOR_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(13), InvertedValue.CounterClockwise_Positive);

    DYEROTOR_CONFIG.supplyCurrentLimit = 40;
    DYEROTOR_CONFIG.enableSupplyCurrentLimit = true;
    DYEROTOR_CONFIG.statorCurrentLimit = 80;
    DYEROTOR_CONFIG.enableStatorCurrentLimit = true;
  }
}
