package frc2026.robot.subsystems.climber;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

  public static final double CLIMBER_REDUCTION = 416.66;

  // Theoretically MAX_HEIGHT / PULLEY_CIRCUMFERENCE, but needs to actually be measured
  public static final double ENCODER_MAX = Units.degreesToRotations(180);

  public static final double ENCODER_MIN = 0.0;

  public static final double REDUCTION = 20;

  public static final SingleJointedArmSim SIM =
      new SingleJointedArmSim(
          DCMotor.getKrakenX44(1),
          CLIMBER_REDUCTION,
          0.0007,
          6,
          ENCODER_MIN,
          ENCODER_MAX,
          false,
          CLIMBER_REDUCTION,
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
        new TalonFXConstants(
            new CANDevice(9, ""), InvertedValue.Clockwise_Positive); // Left Elevator Inside
    CONFIGURATION.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(
              new CANDevice(8, ""),
              InvertedValue.CounterClockwise_Positive), // Left Elevator Outside
          new TalonFXConstants(
              new CANDevice(14, ""), InvertedValue.Clockwise_Positive), // Right Elevator Inside
          new TalonFXConstants(
              new CANDevice(15, ""),
              InvertedValue.CounterClockwise_Positive), // Right Elevator Outside
        };

    CONFIGURATION.neutralMode = NeutralModeValue.Brake;
    CONFIGURATION.sensorToMechRatio = REDUCTION;
    CONFIGURATION.enableSupplyCurrentLimit = true;
    CONFIGURATION.supplyCurrentLimit = 37; // 37
    CONFIGURATION.statorCurrentLimit = 80; // 80
    CONFIGURATION.minUnitsLimit = ENCODER_MIN;
    CONFIGURATION.maxUnitsLimit = ENCODER_MAX;
    CONFIGURATION.cruiseVelocity = 60.0; // 30.0
    CONFIGURATION.acceleration = 40.0;
    CONFIGURATION.slot0 =
        new ScreamPIDConstants(45.0, 0, 0) // 60.0
            .getSlot0Configs(
                new FeedforwardConstants(0, 0.0, 0.3, 0, GravityTypeValue.Elevator_Static));
    // CONFIGURATION.positionThreshold = Climber.heightToRotations(Length.fromInches(0.2)); // 4.0
  }
}
