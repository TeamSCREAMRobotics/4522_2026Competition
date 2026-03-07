package frc2026.tars.subsystems.shooter.kicker;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;

public class KickerConstants {
  public static final TalonFXSubsystemConfiguration KICKER_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    KICKER_CONFIG.name = "Kicker";

    KICKER_CONFIG.codeEnabled = true;
    KICKER_CONFIG.logTelemetry = false;
    KICKER_CONFIG.debugMode = false;

    KICKER_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(16), InvertedValue.Clockwise_Positive);

    KICKER_CONFIG.neutralMode = NeutralModeValue.Coast;

    KICKER_CONFIG.enableSupplyCurrentLimit = true;
    KICKER_CONFIG.supplyCurrentLimit = 20;
  }
}
