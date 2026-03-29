package frc2026.tars.subsystems.hopper;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.MotionMagicConstants;
import edu.wpi.first.math.util.Units;

public class HopperConstants {

  public static final double HOPPER_REDUCTION = 1.25;

  public static final MotionMagicConstants MOTION_MAGIC_CONSTANTS =
      new MotionMagicConstants(2, 0.175, 0);

  public static final MotionMagicConstants FAST_MOTION_MAGIC_CONSTANTS =
      new MotionMagicConstants(2, 0.175, 0);
  public static final MotionMagicConstants SLOW_MOTION_MAGIC_CONSTANTS =
      new MotionMagicConstants(0.5, 0.044, 0);

  public static final TalonFXSubsystemConfiguration HOPPER_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    HOPPER_CONFIG.name = "Hopper";

    HOPPER_CONFIG.codeEnabled = false;
    HOPPER_CONFIG.logTelemetry = false;
    HOPPER_CONFIG.debugMode = false;

    HOPPER_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(11), InvertedValue.CounterClockwise_Positive);

    HOPPER_CONFIG.neutralMode = NeutralModeValue.Brake;
    HOPPER_CONFIG.sensorToMechRatio = HOPPER_REDUCTION;
    HOPPER_CONFIG.feedbackRemoteSensorId = 4;
    HOPPER_CONFIG.enableSupplyCurrentLimit = true;
    HOPPER_CONFIG.supplyCurrentLimit = 15;
    HOPPER_CONFIG.cruiseVelocity = 30.0;
    HOPPER_CONFIG.acceleration = 30.0;
    HOPPER_CONFIG.slot0 =
        new ScreamPIDConstants(2.0, 0, 0)
            .getSlot0Configs(
                new FeedforwardConstants(0, 1.3, 0, 0, GravityTypeValue.Elevator_Static));
    HOPPER_CONFIG.positionThreshold = Units.degreesToRotations(1.0);
  }
}
