package frc2026.tars.commands;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc2026.tars.subsystems.shooter.feeder.Feeder;
import frc2026.tars.subsystems.shooter.rollers.Rollers;

public class IntakeFeed extends Command {
  private static final double SENSOR_BRIDGE_SECONDS = 1.0;
  private static final double INDEX_TIMEOUT_SECONDS = 2.0;

  private final Feeder feeder;
  private final Rollers rollers;
  private final CANrange beam;
  private final CANrange beamOne;

  private Debouncer indexedDebouncer = new Debouncer(0.03, DebounceType.kRising);

  private boolean pieceSeen;
  private double startTime;

  public IntakeFeed(Feeder feeder, Rollers rollers, CANrange beam, CANrange beamOne) {
    this.feeder = feeder;
    this.rollers = rollers;
    this.beam = beam;
    this.beamOne = beamOne;

    addRequirements(feeder, rollers);
  }

  @Override
  public void initialize() {
    indexedDebouncer = new Debouncer(0.03, DebounceType.kRising);
    pieceSeen = beam.getIsDetected().getValue();
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    boolean indexed = beamOne.getIsDetected().getValue();
    double elapsed = Timer.getFPGATimestamp() - startTime;
    pieceSeen = pieceSeen || beam.getIsDetected().getValue();

    if (!indexed && (pieceSeen || elapsed < SENSOR_BRIDGE_SECONDS)) {
      feeder.setVoltage(3.0);
      rollers.setVoltage(1.5);
    } else {
      feeder.setVoltage(0.0);
      rollers.setVoltage(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    rollers.setVoltage(0.0);
    feeder.setVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    double elapsed = Timer.getFPGATimestamp() - startTime;
    return indexedDebouncer.calculate(beamOne.getIsDetected().getValue())
        || elapsed >= INDEX_TIMEOUT_SECONDS
        || (!pieceSeen && elapsed >= SENSOR_BRIDGE_SECONDS);
  }
}
