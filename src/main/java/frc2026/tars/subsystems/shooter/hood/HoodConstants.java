package frc2026.tars.subsystems.shooter.hood;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;
import edu.wpi.first.math.geometry.Rotation2d;

public class HoodConstants {
  public static final double HOOD_REDUCTION = 20.0;

  public static final Rotation2d HOOD_MAX_ANGLE = Rotation2d.fromDegrees(42.786125);
  public static final Rotation2d HOOD_MIN_ANGLE = Rotation2d.fromDegrees(20.786125);

  public static final Rotation2d HOOD_MAX_EXIT_ANGLE = Rotation2d.kCCW_90deg.minus(HOOD_MAX_ANGLE);
  public static final Rotation2d HOOD_MIN_EXIT_ANGLE = Rotation2d.kCCW_90deg.minus(HOOD_MIN_ANGLE);

  public static final Rotation2d HOOD_OFFSET = Rotation2d.fromDegrees(20.786125);

  public static final double MIN_UNITS = 0.0;
  public static final double MAX_UNITS = 0.01; // TODO: Measure

  public static final TalonFXSubsystemConfiguration HOOD_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    HOOD_CONFIG.name = "Hood";
    HOOD_CONFIG.logPrefix = "Hood/";

    HOOD_CONFIG.codeEnabled = true;
    HOOD_CONFIG.logTelemetry = false;
    HOOD_CONFIG.debugMode = false;

    HOOD_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(9), InvertedValue.Clockwise_Positive);

    HOOD_CONFIG.slot0 =
        new ScreamPIDConstants(23.0, 0, 0)
            .getSlot0Configs(new FeedforwardConstants(0, 0.75, 0.0, 0));

    HOOD_CONFIG.neutralMode = NeutralModeValue.Brake;
    HOOD_CONFIG.sensorToMechRatio = HOOD_REDUCTION;
    HOOD_CONFIG.enableSupplyCurrentLimit = true;
    HOOD_CONFIG.supplyCurrentLimit = 35;

    HOOD_CONFIG.acceleration = 120.0;
    HOOD_CONFIG.cruiseVelocity = 30.0;
  }
}
