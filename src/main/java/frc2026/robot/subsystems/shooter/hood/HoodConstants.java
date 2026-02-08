package frc2026.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;

public class HoodConstants {
  // TODO: Put in actual values
  public static final double HOOD_REDUCTION = 0;

  public static final double HOOD_MAX_ANGLE = 0;
  public static final double HOOD_MIN_ANGLE = 0;

  public static final TalonFXSubsystemConfiguration HOOD_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    HOOD_CONFIG.name = "Hood";

    HOOD_CONFIG.codeEnabled = true;
    HOOD_CONFIG.logTelemetry = false;
    HOOD_CONFIG.debugMode = false;

    HOOD_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(10), InvertedValue.CounterClockwise_Positive);

    HOOD_CONFIG.neutralMode = NeutralModeValue.Brake;
    HOOD_CONFIG.rotorToSensorRatio = HOOD_REDUCTION;
    HOOD_CONFIG.enableSupplyCurrentLimit = true;
    HOOD_CONFIG.supplyCurrentLimit = 35;
  }
}
