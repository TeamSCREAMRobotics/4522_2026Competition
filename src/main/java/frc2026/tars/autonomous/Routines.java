package frc2026.tars.autonomous;

import com.teamscreamrobotics.util.BLinePathSequence;
import com.teamscreamrobotics.util.BLinePathSequence.FlipType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.FollowPath;
import frc2026.tars.RobotContainer;
import frc2026.tars.RobotState;
import frc2026.tars.autonomous.commands.AutoIntakeFeed;
import frc2026.tars.commands.IntakeFeed;
import frc2026.tars.controlboard.Controlboard;
import frc2026.tars.subsystems.drivetrain.Drivetrain;
import frc2026.tars.subsystems.drivetrain.DrivetrainConstants;
import frc2026.tars.subsystems.intake.IntakeRollers;
import frc2026.tars.subsystems.intake.IntakeRollers.IntakeRollersGoal;
import frc2026.tars.subsystems.intake.IntakeWrist;
import frc2026.tars.subsystems.intake.IntakeWrist.IntakeWristGoal;
import frc2026.tars.subsystems.shooter.Shooter;
import frc2026.tars.subsystems.shooter.feeder.Feeder;
import frc2026.tars.subsystems.shooter.rollers.Rollers;
import lombok.Getter;

public class Routines {
  private final Shooter shooter;
  private final Drivetrain drivetrain;
  private final RobotState robotState;

  private final Feeder feeder;
  private final Rollers rollers;

  private final FollowPath.Builder pathBuilder;
  private final BLinePathSequence Overbump_Outpost;
  private final BLinePathSequence Overbump_Depot;
  private final IntakeWrist intakeWrist;
  private final IntakeRollers intakeRollers;

  @Getter public SendableChooser<Command> routineChooser;

  public Routines(
      Drivetrain drivetrain,
      Shooter shooter,
      RobotState robotState,
      RobotContainer robotContainer) {
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.robotState = robotState;

    this.feeder = robotContainer.getSubsystems().feeder();
    this.rollers = robotContainer.getSubsystems().rollers();

    this.intakeWrist = robotContainer.getSubsystems().intakeWrist();
    this.intakeRollers = robotContainer.getSubsystems().intakeRollers();

    pathBuilder =
        new FollowPath.Builder(
                drivetrain,
                drivetrain::getEstimatedPose,
                () -> drivetrain.getState().Speeds,
                (speeds) ->
                    drivetrain.setControl(drivetrain.getHelper().getApplyFieldSpeeds(speeds)),
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(3.0, 0.0, 0.0),
                new PIDController(2.0, 0.0, 0.0))
            .withPoseReset(drivetrain::resetPose)
            .withDefaultShouldFlip();

    FollowPath.registerEventTrigger(
        "deployIntake", intakeWrist.applyGoalCommand(IntakeWristGoal.EXTENDED));
    FollowPath.registerEventTrigger(
        "intakeFeed", new IntakeFeed(feeder, rollers, shooter.beam, shooter.beamOne));

    Overbump_Outpost =
        new BLinePathSequence(pathBuilder, FlipType.RotatedField, "Overbump_One", "Overbump_Two");

    Overbump_Depot =
        new BLinePathSequence(pathBuilder, FlipType.RotatedField, "Overbump_One", "Overbump_Two")
            .mirror();

    routineChooser = new SendableChooser<>();
    routineChooser.setDefaultOption("Do Nothing", null);
    routineChooser.addOption("Overbump Outpost", OverbumpOutpost());
    routineChooser.addOption("Overbump Depo", OverbumpDepo());

    SmartDashboard.putData("AutoChooser", routineChooser);
  }

  private ParallelRaceGroup Shoot() {
    return new ParallelRaceGroup(
        drivetrain
            .applyRequest(
                () ->
                    drivetrain
                        .getHelper()
                        .getFacingAngleProfiled(
                            Controlboard.getTranslation(),
                            robotState.getDrivetrainTarget(),
                            DrivetrainConstants.headingControllerProfiled))
            .beforeStarting(() -> drivetrain.resetHeadingController()),
        intakeRollers.applyGoalCommand(IntakeRollersGoal.AUTOINTAKE),
        shooter.autoShoot());
  }

  public Command OverbumpOutpost() {
    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            new AutoIntakeFeed(feeder, rollers, intakeRollers, shooter.beam, shooter.beamOne),
            Overbump_Outpost.getNext()),
        Shoot(),
        new ParallelRaceGroup(
            new AutoIntakeFeed(feeder, rollers, intakeRollers, shooter.beam, shooter.beamOne),
            Overbump_Outpost.getNext()),
        Shoot());
  }

  public Command OverbumpDepo() {
    return new SequentialCommandGroup(
        new ParallelRaceGroup(
            new AutoIntakeFeed(feeder, rollers, intakeRollers, shooter.beam, shooter.beamOne),
            Overbump_Depot.getNext()),
        Shoot(),
        new ParallelRaceGroup(
            new AutoIntakeFeed(feeder, rollers, intakeRollers, shooter.beam, shooter.beamOne),
            Overbump_Depot.getNext()),
        Shoot());
  }
}
