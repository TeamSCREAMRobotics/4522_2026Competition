package frc2026.tars.subsystems.shooter.flywheel;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;

public class FlywheelConstants {
  public static final double FLYWHEEL_REDUCTION = 1.0;

  public static final Length FLYWHEEL_CIRCUMFERENCE = Length.fromInches(4.0 * Math.PI);

  public static final Length FLYWHEEL_DIAMETER = Length.fromInches(4.0);
  public static final Length FLYWHEEL_RADIUS = Length.fromInches(FLYWHEEL_DIAMETER.getInches() / 2);

  public static final TalonFXSubsystemConfiguration FLYWHEEL_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    FLYWHEEL_CONFIG.name = "Flywheel";

    FLYWHEEL_CONFIG.codeEnabled = true;
    FLYWHEEL_CONFIG.logTelemetry = false;
    FLYWHEEL_CONFIG.debugMode = false;

    FLYWHEEL_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(10), InvertedValue.CounterClockwise_Positive);
    FLYWHEEL_CONFIG.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(11), InvertedValue.Clockwise_Positive)
        };
    FLYWHEEL_CONFIG.slot0 =
        new ScreamPIDConstants(4.5, 0.0, 0.0)
            .getSlot0Configs(new FeedforwardConstants(0.15, 0.17, 0.0, 0.0));

    FLYWHEEL_CONFIG.neutralMode = NeutralModeValue.Coast;

    FLYWHEEL_CONFIG.enableSupplyCurrentLimit = true;
    FLYWHEEL_CONFIG.supplyCurrentLimit = 20;
    FLYWHEEL_CONFIG.sensorToMechRatio = FLYWHEEL_REDUCTION;

    FLYWHEEL_CONFIG.peakForwardTorqueCurrent = 40.0;
    FLYWHEEL_CONFIG.peakReverseTorqueCurrent = 0.0;
  }
}
