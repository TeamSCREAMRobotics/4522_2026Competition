package frc2026.tars.subsystems.shooter.hood;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class HoodConstants {
  // TODO: Put in actual values
  public static final double HOOD_REDUCTION = 2.0;

  public static final Rotation2d HOOD_MAX_ANGLE = new Rotation2d(Units.degreesToRadians(42.786125));
  public static final Rotation2d HOOD_MIN_ANGLE = new Rotation2d(Units.degreesToRadians(20.786125));

  public static final TalonFXSubsystemConfiguration HOOD_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    HOOD_CONFIG.name = "Hood";

    HOOD_CONFIG.codeEnabled = true;
    HOOD_CONFIG.logTelemetry = false;
    HOOD_CONFIG.debugMode = false;

    HOOD_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(9), InvertedValue.CounterClockwise_Positive);

    HOOD_CONFIG.neutralMode = NeutralModeValue.Brake;
    HOOD_CONFIG.rotorToSensorRatio = HOOD_REDUCTION;
    HOOD_CONFIG.enableSupplyCurrentLimit = true;
    HOOD_CONFIG.supplyCurrentLimit = 35;

    HOOD_CONFIG.acceleration = 120.0;
  }
}
