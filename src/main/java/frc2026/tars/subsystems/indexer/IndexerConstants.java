package frc2026.tars.subsystems.indexer;

import com.ctre.phoenix6.signals.InvertedValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.*;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;

public class IndexerConstants {
  private static final double SPINDEXER_REDUCTION = 0.0;

  private static final double FEEDER_REDUCTION = 0.0;

  public static final TalonFXSubsystemConfiguration SPINDEXER_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    SPINDEXER_CONFIG.name = "Spindexer";

    SPINDEXER_CONFIG.codeEnabled = true;
    SPINDEXER_CONFIG.logTelemetry = false;
    SPINDEXER_CONFIG.logTelemetry = false;
    SPINDEXER_CONFIG.debugMode = false;

    SPINDEXER_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(13), InvertedValue.CounterClockwise_Positive);

    SPINDEXER_CONFIG.supplyCurrentLimit = 40;
    SPINDEXER_CONFIG.enableSupplyCurrentLimit = true;
    SPINDEXER_CONFIG.statorCurrentLimit = 80;
    SPINDEXER_CONFIG.enableStatorCurrentLimit = true;
  }

  public static final TalonFXSubsystemConfiguration FEEDER_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    FEEDER_CONFIG.name = "Feeder";

    FEEDER_CONFIG.codeEnabled = true;
    FEEDER_CONFIG.logTelemetry = false;
    FEEDER_CONFIG.logTelemetry = false;
    FEEDER_CONFIG.debugMode = false;

    FEEDER_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(12), InvertedValue.CounterClockwise_Positive);

    FEEDER_CONFIG.supplyCurrentLimit = 40;
    FEEDER_CONFIG.enableSupplyCurrentLimit = true;
    FEEDER_CONFIG.statorCurrentLimit = 80;
    FEEDER_CONFIG.enableStatorCurrentLimit = true;
  }
}
