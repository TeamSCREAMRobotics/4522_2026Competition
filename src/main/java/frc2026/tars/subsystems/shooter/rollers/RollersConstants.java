package frc2026.tars.subsystems.shooter.rollers;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;

public class RollersConstants {

  public static final TalonFXSubsystemConfiguration ROLLERS_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    ROLLERS_CONFIG.name = "Rollers";

    ROLLERS_CONFIG.codeEnabled = true;
    ROLLERS_CONFIG.logTelemetry = false;
    ROLLERS_CONFIG.debugMode = false;

    ROLLERS_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(6), InvertedValue.CounterClockwise_Positive);

    ROLLERS_CONFIG.neutralMode = NeutralModeValue.Coast;

    ROLLERS_CONFIG.enableSupplyCurrentLimit = true;
    ROLLERS_CONFIG.supplyCurrentLimit = 40;
  }
}
