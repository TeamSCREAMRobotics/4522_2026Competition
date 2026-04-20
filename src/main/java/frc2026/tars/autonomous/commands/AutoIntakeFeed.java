package frc2026.tars.autonomous.commands;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc2026.tars.subsystems.intake.IntakeRollers;
import frc2026.tars.subsystems.shooter.feeder.Feeder;
import frc2026.tars.subsystems.shooter.rollers.Rollers;

public class AutoIntakeFeed extends Command {

  private final Feeder feeder;
  private final Rollers rollers;
  private final IntakeRollers intakeRollers;
  private final CANrange beam;
  private final CANrange beamOne;

  private Debouncer beamDebouncer = new Debouncer(0.03, DebounceType.kRising);

  private boolean hasBallKnowledge;

  public AutoIntakeFeed(
      Feeder feeder,
      Rollers rollers,
      IntakeRollers intakeRollers,
      CANrange beam,
      CANrange beamOne) {
    this.feeder = feeder;
    this.rollers = rollers;
    this.intakeRollers = intakeRollers;
    this.beam = beam;
    this.beamOne = beamOne;

    addRequirements(feeder, rollers, intakeRollers);
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
    if (!hasBallKnowledge
        && beam.getIsDetected().getValue()
        && !beamOne.getIsDetected().getValue()) {
      intakeRollers.setVoltage(12.0);
      feeder.setVoltage(3.0);
      rollers.setVoltage(1.5);
    } else if (!hasBallKnowledge
        && beam.getIsDetected().getValue()
        && beamDebouncer.calculate(beamOne.getIsDetected().getValue())) {
      intakeRollers.setVoltage(12.0);
      feeder.setVoltage(0.0);
      rollers.setVoltage(0.0);
    } else if (hasBallKnowledge
        && !beam.getIsDetected().getValue()
        && !beamOne.getIsDetected().getValue()) {
      intakeRollers.setVoltage(12.0);
      feeder.setVoltage(3.0);
      rollers.setVoltage(1.5);
    } else {
      intakeRollers.setVoltage(12.0);
      feeder.setVoltage(0.0);
      rollers.setVoltage(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeRollers.setVoltage(0.0);
    rollers.setVoltage(0.0);
    feeder.setVoltage(0.0);
  }
}
