// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2026.robot.subsystems.climber.constants;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.teamscreamrobotics.data.Length;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.CANDevice;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXConstants;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import com.teamscreamrobotics.pid.ScreamPIDConstants;
import com.teamscreamrobotics.pid.ScreamPIDConstants.FeedforwardConstants;
import frc2026.robot.subsystems.climber.ClimbElevator;

/** Add your docs here. */
public final class ElevatorConstants {

  // As measured from wrist pivot axis
  public static final Length MIN_HEIGHT_FROM_FLOOR = Length.fromInches(10.7125);
  public static final Length MAX_HEIGHT_FROM_FLOOR = Length.fromInches(89.825);

  public static final double MIN_HEIGHT = 0.0;
  public static final Length MAX_HEIGHT =
      MAX_HEIGHT_FROM_FLOOR.minus(MIN_HEIGHT_FROM_FLOOR); // 79.1125

  public static final Length PULLEY_DIAMETER = Length.fromInches(2.256);
  public static final Length PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER.times(Math.PI);

  // Theoretically MAX_HEIGHT / PULLEY_CIRCUMFERENCE, but needs to actually be measured
  public static final double ENCODER_MAX =
      MAX_HEIGHT.getInches() / PULLEY_CIRCUMFERENCE.getInches();
  public static final double ENCODER_MIN = 0.0;

  public static final double REDUCTION = 3.125;

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Elevator";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;
    CONFIGURATION.debugMode = false;

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
    CONFIGURATION.positionThreshold =
        ClimbElevator.heightToRotations(Length.fromInches(0.2)); // 4.0
  }
}
