// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2026.tars.subsystems.indexer;

import com.ctre.phoenix6.signals.InvertedValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.*;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;

/** Add your docs here. */
public class IndexerConstants {
  private static final double INDEXER_REDUCTION = 0.0;
  private static final double PIPELINE_REDUCTION = 0.0;

  private static final TalonFXSubsystemConfiguration INDEXER_CONFIG =
      new TalonFXSubsystemConfiguration();

  private static final TalonFXSubsystemConfiguration PIPELINE_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    INDEXER_CONFIG.name = "Indexer";

    INDEXER_CONFIG.codeEnabled = false;
    INDEXER_CONFIG.logTelemetry = false;
    INDEXER_CONFIG.logTelemetry = false;
    INDEXER_CONFIG.debugMode = false;

    INDEXER_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(13, ""), InvertedValue.Clockwise_Positive);
    INDEXER_CONFIG.sensorToMechRatio = INDEXER_REDUCTION;
    INDEXER_CONFIG.supplyCurrentLimit = 40;
    INDEXER_CONFIG.enableSupplyCurrentLimit = true;
    INDEXER_CONFIG.statorCurrentLimit = 80;
    INDEXER_CONFIG.enableStatorCurrentLimit = true;
  }

  static {
    PIPELINE_CONFIG.name = "PIPELINE";

    PIPELINE_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(23), InvertedValue.Clockwise_Positive);

    PIPELINE_CONFIG.supplyCurrentLimit = 40;
    PIPELINE_CONFIG.enableSupplyCurrentLimit = true;
    PIPELINE_CONFIG.sensorToMechRatio = PIPELINE_REDUCTION;
  }
}
