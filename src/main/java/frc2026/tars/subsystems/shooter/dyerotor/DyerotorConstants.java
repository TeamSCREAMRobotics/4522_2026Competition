package frc2026.tars.subsystems.shooter.dyerotor;

import com.ctre.phoenix6.signals.InvertedValue;
import com.teamscreamrobotics.drivers.TalonFXSubsystem.*;
import com.teamscreamrobotics.sim.SimWrapper;
import com.teamscreamrobotics.util.SimUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class DyerotorConstants {
  public static final TalonFXSubsystemConfiguration DYEROTOR_CONFIG =
      new TalonFXSubsystemConfiguration();

  private static final DCMotorSim sim = SimUtil.createDCMotorSim(DCMotor.getKrakenX60(1), 1.0, 0.1);

  static {
    DYEROTOR_CONFIG.name = "Spindexer";

    DYEROTOR_CONFIG.codeEnabled = true;
    DYEROTOR_CONFIG.logTelemetry = false;
    DYEROTOR_CONFIG.debugMode = false;

    DYEROTOR_CONFIG.masterConstants =
        new TalonFXConstants(new CANDevice(13), InvertedValue.CounterClockwise_Positive);

    DYEROTOR_CONFIG.simConstants =
        new TalonFXSubsystemSimConstants(new SimWrapper(sim), 1.0, new PIDController(1, 0, 0));

    //  DYEROTOR_CONFIG.slaveConstants =
    //      new TalonFXConstants[] {
    //       new TalonFXConstants(new CANDevice(16), InvertedValue.Clockwise_Positive)
    //      };

    DYEROTOR_CONFIG.supplyCurrentLimit = 30;
    DYEROTOR_CONFIG.enableSupplyCurrentLimit = true;
    DYEROTOR_CONFIG.statorCurrentLimit = 40;
    DYEROTOR_CONFIG.enableStatorCurrentLimit = true;
  }
}
