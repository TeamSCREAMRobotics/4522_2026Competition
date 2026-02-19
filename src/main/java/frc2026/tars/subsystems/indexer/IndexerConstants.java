package frc2026.tars.subsystems.indexer;

import com.ctre.phoenix6.signals.InvertedValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.*;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;

public class IndexerConstants {
  private static final double SPINDEXER_REDUCTION = 0.0;

  private static final double FEEDER_REDUCTION = 0.0;

  public static final TalonFXSubsystemConfiguration INDEXER_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    INDEXER_CONFIG.name = "Indexer";

    INDEXER_CONFIG.codeEnabled = false;
    INDEXER_CONFIG.logTelemetry = false;
    INDEXER_CONFIG.logTelemetry = false;
    INDEXER_CONFIG.debugMode = false;

    INDEXER_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(13), InvertedValue.Clockwise_Positive);
    INDEXER_CONFIG.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(12), InvertedValue.Clockwise_Positive)
        };

    INDEXER_CONFIG.sensorToMechRatio = SPINDEXER_REDUCTION;
    INDEXER_CONFIG.supplyCurrentLimit = 40;
    INDEXER_CONFIG.enableSupplyCurrentLimit = true;
    INDEXER_CONFIG.statorCurrentLimit = 80;
    INDEXER_CONFIG.enableStatorCurrentLimit = true;
  }

  public static final TalonFXSubsystemConfiguration FEEDER_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    FEEDER_CONFIG.name = "Feeder";

    FEEDER_CONFIG.codeEnabled = false;
    FEEDER_CONFIG.logTelemetry = false;
    FEEDER_CONFIG.logTelemetry = false;
    FEEDER_CONFIG.debugMode = false;

    FEEDER_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(12), InvertedValue.Clockwise_Positive);

    FEEDER_CONFIG.sensorToMechRatio = FEEDER_REDUCTION;
    FEEDER_CONFIG.supplyCurrentLimit = 40;
    FEEDER_CONFIG.enableSupplyCurrentLimit = true;
    FEEDER_CONFIG.statorCurrentLimit = 80;
    FEEDER_CONFIG.enableStatorCurrentLimit = true;
  }
}
