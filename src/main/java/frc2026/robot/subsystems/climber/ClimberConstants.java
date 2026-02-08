package frc2026.robot.subsystems.climber;

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

public final class ClimberConstants {

  public static final Length PULLEY_RADIUS = Length.fromInches(0);
  public static final Length PULLEY_DIAMETER = PULLEY_RADIUS.times(2);
  public static final Length PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER.times(Math.PI);

  public static final double ENCODER_MAX = Units.degreesToRotations(180);

  public static final double ENCODER_MIN = 0.0;

  public static final double REDUCTION = 416.66;

  public static final SingleJointedArmSim SIM =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(1),
          REDUCTION,
          0.0007,
          6,
          ENCODER_MIN,
          ENCODER_MAX,
          false,
          REDUCTION,
          null);
  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(5.0, 0.0, 0.0);

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Climber";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;
    CONFIGURATION.debugMode = false;

    CONFIGURATION.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM, REDUCTION),
            REDUCTION,
            SIM_GAINS.getProfiledPIDController(new Constraints(1.5 * 0.7, 0.7)),
            false,
            true);

    CONFIGURATION.masterConstants =
        new TalonFXConstants(new CANDevice(9, ""), InvertedValue.Clockwise_Positive);

    CONFIGURATION.neutralMode = NeutralModeValue.Brake;
    CONFIGURATION.sensorToMechRatio = REDUCTION;
    CONFIGURATION.enableSupplyCurrentLimit = true;
    CONFIGURATION.supplyCurrentLimit = 37;
    CONFIGURATION.statorCurrentLimit = 80;
    CONFIGURATION.minUnitsLimit = ENCODER_MIN;
    CONFIGURATION.maxUnitsLimit = ENCODER_MAX;
    CONFIGURATION.cruiseVelocity = 60.0;
    CONFIGURATION.acceleration = 40.0;
    CONFIGURATION.slot0 =
        new ScreamPIDConstants(45.0, 0, 0)
            .getSlot0Configs(
                new FeedforwardConstants(0, 0.0, 0.3, 0, GravityTypeValue.Elevator_Static));
  }
}
