// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2026.robot.subsystems.climber.constants;

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

/** Add your docs here. */
public class HookConstants {

  public static final Length MANIPULATOR_LENGTH = Length.fromInches(18.668462);

  public static final double WRIST_REDUCTION = 30.0;
  public static final double ROLLERS_REDUCTION = 2.25;

  public static final double ACQUIRED_PIECE_THRESHOLD = 0.0;

  public static final SingleJointedArmSim SIM =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          WRIST_REDUCTION,
          0.00490209781964,
          Units.inchesToMeters(20.5),
          0.0,
          Math.PI / 2.0,
          false,
          Math.PI / 2.0);
  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(50.0, 0.0, 50.0);

  public static final TalonFXSubsystemConfiguration WRIST_CONFIG =
      new TalonFXSubsystemConfiguration();

  static {
    WRIST_CONFIG.name = "Wrist";

    WRIST_CONFIG.codeEnabled = true;
    WRIST_CONFIG.logTelemetry = false;
    WRIST_CONFIG.debugMode = false;

    WRIST_CONFIG.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM, WRIST_REDUCTION),
            WRIST_REDUCTION,
            SIM_GAINS.getProfiledPIDController(new Constraints(0.5, 0.1)),
            true,
            true);

    WRIST_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(10), InvertedValue.CounterClockwise_Positive);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.625;
    config.MagnetSensor.MagnetOffset = -0.139404296875 + 0.25; // -0.133544921875$
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    WRIST_CONFIG.cancoderConstants = new CANCoderConstants(new CANDevice(4), config);

    WRIST_CONFIG.maxUnitsLimit = 0.25;
    WRIST_CONFIG.minUnitsLimit = 0.0;

    WRIST_CONFIG.neutralMode = NeutralModeValue.Brake;
    WRIST_CONFIG.rotorToSensorRatio = WRIST_REDUCTION;
    WRIST_CONFIG.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    WRIST_CONFIG.feedbackRemoteSensorId = 4;
    WRIST_CONFIG.enableSupplyCurrentLimit = true;
    WRIST_CONFIG.supplyCurrentLimit = 40;
    WRIST_CONFIG.cruiseVelocity = 30.0;
    WRIST_CONFIG.acceleration = 30.0;
    WRIST_CONFIG.slot0 =
        new ScreamPIDConstants(60.0, 0, 0) // 42.5
            .getSlot0Configs(new FeedforwardConstants(0, 0, 0.6, 0, GravityTypeValue.Arm_Cosine));
    WRIST_CONFIG.positionThreshold = Units.degreesToRotations(3.0);
  }
}
