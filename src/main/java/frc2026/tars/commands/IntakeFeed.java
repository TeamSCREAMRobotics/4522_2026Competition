package frc2026.tars.commands;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc2026.tars.subsystems.shooter.feeder.Feeder;
import frc2026.tars.subsystems.shooter.rollers.Rollers;

public class IntakeFeed extends Command {
  private final Feeder feeder;
  private final Rollers rollers;
  private final CANrange beam;
  private final CANrange beamOne;

  private Debouncer beamDebouncer = new Debouncer(0.03, DebounceType.kRising);

  private boolean hasBallKnowledge;

  public IntakeFeed(Feeder feeder, Rollers rollers, CANrange beam, CANrange beamOne) {
    this.feeder = feeder;
    this.rollers = rollers;
    this.beam = beam;
    this.beamOne = beamOne;

    addRequirements(feeder, rollers);
  }

  @Override
  public void initialize() {
    if (beam.getIsDetected().getValue()) {
      hasBallKnowledge = true;
    } else {
      hasBallKnowledge = false;
    }
  }

  @Override
  public void execute() {
    if (hasBallKnowledge) {
      feeder.setVoltage(3.0);
      rollers.setVoltage(1.5);
    }
  }

  @Override
  public void end(boolean interrupted) {
    rollers.setVoltage(0.0);
    feeder.setVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return beamDebouncer.calculate(beamOne.getIsDetected().getValue());
  }
}
