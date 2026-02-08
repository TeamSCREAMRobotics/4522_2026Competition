// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2026.robot.subsystems.indexer;

import com.teamscreamrobotics.drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class Indexer extends TalonFXSubsystem {

  public Indexer(TalonFXSubsystemConfiguration config) {
    super(config);
  }

  public enum IndexerGoal implements TalonFXSubsystemGoal {
    STOP(() -> 0.0, ControlType.VOLTAGE),
    RUN(()-> 4.5, ControlType.VOLTAGE);

    public final DoubleSupplier voltage;
    public final ControlType controlType;

    private IndexerGoal(DoubleSupplier voltage, ControlType controlType) {
      this.voltage = voltage;
      this.controlType = controlType;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
    }

    @Override
    public DoubleSupplier target() {
      return voltage;
    }
  }

}
