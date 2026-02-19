package frc2026.tars.subsystems.indexer;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc2026.tars.subsystems.indexer.Feeder.FeederGoal;
import frc2026.tars.subsystems.indexer.Spindexer.SpindexerGoal;

public class Indexer {
  private static final CANrange beam = new CANrange(0);
  private final Spindexer spindexer = new Spindexer(IndexerConstants.INDEXER_CONFIG);
  private final Feeder feeder = new Feeder(IndexerConstants.FEEDER_CONFIG);

  public static boolean noFuel() {
    if (Units.metersToInches(beam.getDistance().getValueAsDouble()) < 3.0) {
      return false;
    } else {
      Timer.delay(0.2);
      return true;
    }
  }

  public Command runUntilNoFuel() {
    return new SequentialCommandGroup(
        Commands.parallel(spindexer.applyGoalCommand(SpindexerGoal.RUN))
            .withDeadline(new WaitUntilCommand(() -> noFuel())),
        spindexer.applyGoalCommand(SpindexerGoal.STOP));
  }

  public Command run(SpindexerGoal spinGoal, FeederGoal feedGoal) {
    return Commands.parallel(
        spindexer.applyGoalCommand(spinGoal), feeder.applyGoalCommand(feedGoal));
  }
}
