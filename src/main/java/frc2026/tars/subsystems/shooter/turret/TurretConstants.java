package frc2026.tars.subsystems.shooter.turret;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;
import com.teamscreamrobotics.sim.SimWrapper;
import com.teamscreamrobotics.util.SimUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretConstants {

  public static final double REDUCTION = 45.0;
  public static final double MIN_ROT_DEG = -50.0;
  public static final double MAX_ROT_DEG = 350.0;
  // public static final double MIN_ROT_DEG = -178.0;
  // public static final double MAX_ROT_DEG = 238.0;

  public static final boolean ENABLE_SOFTWARE_LIMIT = true;
  public static final double FORWARD_SOFTWARE_LIMIT = MAX_ROT_DEG / 360.0;
  public static final double BACKWARD_SOFTWARE_LIMIT = MIN_ROT_DEG / 360.0;

  public static final double MAGNITUDE = 0.95;

  public static final DCMotor DC_MOTOR = DCMotor.getKrakenX60(1);
  public static final int CAN_ID = 8;
  public static final double kP = 20.0; // 47.5;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kS = 0.25;
  public static final double kV = 0.0;
  public static final double kA = 0.0;
  public static final double MAX_VEL = 0.01; // rot/s was 30.0
  public static final double MAX_ACCEL = 0.0; // was 10.0, changed to keep chain from breaking
  public static final boolean BRAKE_MODE = true;
  public static final boolean ENABLE_STATOR_LIMIT = true;
  public static final int STATOR_CURRENT_LIMIT = 40;
  public static final boolean ENABLE_SUPPLY_LIMIT = true;
  public static final int SUPPLY_CURRENT_LIMIT = 40;

  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(40.0, 0.0, 0.0);

  public static final DCMotorSim SIM = SimUtil.createDCMotorSim(DC_MOTOR, REDUCTION, 0.01);

  public static final TalonFXSubsystemConfiguration TURRET_CONFIG =
      new TalonFXSubsystemConfiguration();

  public static final double LATENCY = 0.15; // TODO: Tune this constant

  static {
    TURRET_CONFIG.name = "Turret";

    TURRET_CONFIG.codeEnabled = true;
    TURRET_CONFIG.logTelemetry = false;
    TURRET_CONFIG.debugMode = false;

    TURRET_CONFIG.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM), REDUCTION, SIM_GAINS.getPIDController());

    TURRET_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(CAN_ID), InvertedValue.CounterClockwise_Positive);

    // Set current limits
    TURRET_CONFIG.statorCurrentLimit = TurretConstants.STATOR_CURRENT_LIMIT;
    TURRET_CONFIG.enableStatorCurrentLimit = TurretConstants.ENABLE_STATOR_LIMIT;
    TURRET_CONFIG.supplyCurrentLimit = TurretConstants.SUPPLY_CURRENT_LIMIT;
    TURRET_CONFIG.enableSupplyCurrentLimit = TurretConstants.ENABLE_SUPPLY_LIMIT;

    // Set brake mode
    TURRET_CONFIG.neutralMode = NeutralModeValue.Coast;

    // Set motor rotation limits
    TURRET_CONFIG.maxUnitsLimit = FORWARD_SOFTWARE_LIMIT;
    TURRET_CONFIG.minUnitsLimit = BACKWARD_SOFTWARE_LIMIT;

    // Apply gear ratio
    TURRET_CONFIG.sensorToMechRatio = REDUCTION;

    TURRET_CONFIG.cruiseVelocity = MAX_VEL;
    TURRET_CONFIG.acceleration = MAX_ACCEL;
    TURRET_CONFIG.slot0 =
        new ScreamPIDConstants(kP, kI, kD)
            .getSlot0Configs(new FeedforwardConstants(kV, kS, 0.0, kA));
  }
}
