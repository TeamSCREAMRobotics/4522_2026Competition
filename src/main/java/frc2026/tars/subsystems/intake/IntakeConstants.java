package frc2026.tars.subsystems.intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANCoderConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;
import com.teamscreamrobotics.sim.SimWrapper;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeConstants {
  // TODO: Put actual values
  public static final Length INTAKE_LENGTH = Length.fromInches(13.375);

  // Intake Wrist Reduction
  public static final double INTAKE_REDUCTION = 30.0;
  // Intake Roller Reduction
  public static final double ROLLERS_REDUCTION = 1.5;

  public static final double ACQUIRED_PIECE_THRESHOLD = 0.0;

  public static final SingleJointedArmSim SIM =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          INTAKE_REDUCTION,
          0.0490209781964,
          // 40
          Units.inchesToMeters(20.5),
          0.0,
          Units.degreesToRadians(112),
          true,
          Math.PI / 2.0);
  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(50.0, 0.0, 50.0);

  public static final TalonFXSubsystemConfiguration WRIST_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    WRIST_CONFIG.name = "Intake";

    WRIST_CONFIG.codeEnabled = true;
    WRIST_CONFIG.logTelemetry = true;
    WRIST_CONFIG.debugMode = true;

    WRIST_CONFIG.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM, INTAKE_REDUCTION),
            INTAKE_REDUCTION,
            SIM_GAINS.getProfiledPIDController(new Constraints(0.5, 0.1)),
            false,
            true);

    WRIST_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(10), InvertedValue.CounterClockwise_Positive);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.625;
    config.MagnetSensor.MagnetOffset = -0.4853515625 + 0.25;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    WRIST_CONFIG.cancoderConstants = new CANCoderConstants(new CANDevice(4), config);

    WRIST_CONFIG.neutralMode = NeutralModeValue.Brake;
    WRIST_CONFIG.rotorToSensorRatio = INTAKE_REDUCTION;
    WRIST_CONFIG.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    WRIST_CONFIG.feedbackRemoteSensorId = 4;
    WRIST_CONFIG.enableSupplyCurrentLimit = true;
    WRIST_CONFIG.supplyCurrentLimit = 40;
    WRIST_CONFIG.cruiseVelocity = 30.0;
    WRIST_CONFIG.acceleration = 30.0;
    WRIST_CONFIG.slot0 =
        new ScreamPIDConstants(42.5, 0, 0)
            .getSlot0Configs(new FeedforwardConstants(0, 0, 0.6, 0, GravityTypeValue.Arm_Cosine));
    WRIST_CONFIG.positionThreshold = Units.degreesToRotations(3.0);
  }

  public static final TalonFXSubsystemConfiguration ROLLERS_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    ROLLERS_CONFIG.name = "IntakeRollers";

    ROLLERS_CONFIG.codeEnabled = true;
    ROLLERS_CONFIG.logTelemetry = false;

    ROLLERS_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(11), InvertedValue.Clockwise_Positive);

    ROLLERS_CONFIG.enableSupplyCurrentLimit = true;
    ROLLERS_CONFIG.supplyCurrentLimit = 20;
    ROLLERS_CONFIG.sensorToMechRatio = ROLLERS_REDUCTION;
  }
}
