package frc2026.tars.subsystems.intake;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.data.Length;
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
  public static final double INTAKE_REDUCTION = 64.8;
  // Intake Roller Reduction
  public static final double ROLLERS_REDUCTION = 2.5;

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

  public static final TalonFXSubsystemConfiguration INTAKE_WRIST_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    INTAKE_WRIST_CONFIG.name = "IntakeWrist";

    INTAKE_WRIST_CONFIG.codeEnabled = true;
    INTAKE_WRIST_CONFIG.logTelemetry = true;
    INTAKE_WRIST_CONFIG.debugMode = true;

    INTAKE_WRIST_CONFIG.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM, INTAKE_REDUCTION),
            INTAKE_REDUCTION,
            SIM_GAINS.getProfiledPIDController(new Constraints(0.5, 0.1)),
            false,
            true);

    INTAKE_WRIST_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(14), InvertedValue.CounterClockwise_Positive);

    INTAKE_WRIST_CONFIG.neutralMode = NeutralModeValue.Brake;
    INTAKE_WRIST_CONFIG.rotorToSensorRatio = INTAKE_REDUCTION;
    INTAKE_WRIST_CONFIG.feedbackRemoteSensorId = 4;
    INTAKE_WRIST_CONFIG.enableSupplyCurrentLimit = true;
    INTAKE_WRIST_CONFIG.supplyCurrentLimit = 20;
    INTAKE_WRIST_CONFIG.statorCurrentLimit = 20;
    INTAKE_WRIST_CONFIG.cruiseVelocity = 30.0;
    INTAKE_WRIST_CONFIG.acceleration = 30.0;
    INTAKE_WRIST_CONFIG.slot0 =
        new ScreamPIDConstants(4.5, 0, 0)
            .getSlot0Configs(new FeedforwardConstants(0, 0, 0, 0, GravityTypeValue.Arm_Cosine));
    INTAKE_WRIST_CONFIG.positionThreshold = Units.degreesToRotations(3.0);
  }

  public static final TalonFXSubsystemConfiguration ROLLERS_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    ROLLERS_CONFIG.name = "IntakeRollers";

    ROLLERS_CONFIG.codeEnabled = false;
    ROLLERS_CONFIG.logTelemetry = false;

    ROLLERS_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(15), InvertedValue.Clockwise_Positive);

    ROLLERS_CONFIG.enableSupplyCurrentLimit = true;
    ROLLERS_CONFIG.supplyCurrentLimit = 20;
    ROLLERS_CONFIG.sensorToMechRatio = ROLLERS_REDUCTION;
  }
}
