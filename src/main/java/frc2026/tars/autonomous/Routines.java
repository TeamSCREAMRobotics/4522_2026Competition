package frc2026.tars.autonomous;

import com.teamscreamrobotics.util.BLinePathSequence;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.BLine.FlippingUtil.FieldSymmetry;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.Waypoint;
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
import java.util.ArrayList;
import java.util.Optional;
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

  private final BLinePathSequence TestPath;

  private final IntakeWrist intakeWrist;
  private final IntakeRollers intakeRollers;

  @Getter public SendableChooser<Command> routineChooser;

  @Getter private static BLinePathSequence currentSequence;

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
                    drivetrain.setControl(drivetrain.getHelper().getApplyRobotSpeeds(speeds)),
                new PIDController(7.0, 0.0, 0.0),
                new PIDController(5.0, 0.0, 0.0),
                new PIDController(2.0, 0.0, 0.0))
            .withDefaultShouldFlip();

    FollowPath.registerEventTrigger(
        "deployIntake", intakeWrist.applyGoalCommand(IntakeWristGoal.EXTENDED));
    FollowPath.registerEventTrigger(
        "intakeFeed", new IntakeFeed(feeder, rollers, shooter.beam, shooter.beamOne));

    Overbump_Outpost =
        new BLinePathSequence(
            pathBuilder, FieldSymmetry.kRotational, "OverBump_Test", "Overbump_Two");

    Overbump_Depot = Overbump_Outpost.mirror();

    TestPath = new BLinePathSequence(pathBuilder, FieldSymmetry.kRotational, "test_test");

    routineChooser = new SendableChooser<>();
    routineChooser.setDefaultOption("Do Nothing", Commands.none().withName("Do Nothing"));
    routineChooser.addOption("Overbump Outpost", OverbumpOutpost().withName("Overbump Outpost"));
    routineChooser.addOption("Overbump Depot", OverbumpDepot().withName("Overbump Depot"));
    routineChooser.addOption("TestPath", Test().withName("Test"));

    SmartDashboard.putData("AutoChooser", routineChooser);
  }

  private Command resetPose(BLinePathSequence sequence) {
    return Commands.runOnce(() -> drivetrain.resetPose(sequence.getStartingPose()));
  }

  private Command Shoot() {
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
            shooter.autoShoot())
        .andThen(intakeWrist.instantApplyGoalCommand(IntakeWristGoal.EXTENDED));
  }

  public Command OverbumpOutpost() {
    currentSequence = Overbump_Outpost;
    logPoints("OverbumpOutpost", currentSequence);
    return new SequentialCommandGroup(
        resetPose(Overbump_Outpost),
        intakeWrist.instantApplyGoalCommand(IntakeWristGoal.EXTENDED),
        new ParallelRaceGroup(
            new AutoIntakeFeed(feeder, rollers, intakeRollers, shooter.beam, shooter.beamOne),
            Overbump_Outpost.getNext()),
        Shoot(),
        new ParallelRaceGroup(
            new AutoIntakeFeed(feeder, rollers, intakeRollers, shooter.beam, shooter.beamOne),
            Overbump_Outpost.getNext()),
        Shoot());
  }

  public Command OverbumpDepot() {
    currentSequence = Overbump_Depot;
    logPoints("OverbumpDepot", currentSequence);
    return new SequentialCommandGroup(
        resetPose(Overbump_Depot),
        intakeWrist.instantApplyGoalCommand(IntakeWristGoal.EXTENDED),
        new ParallelRaceGroup(
            new AutoIntakeFeed(feeder, rollers, intakeRollers, shooter.beam, shooter.beamOne),
            Overbump_Depot.getNext()),
        Shoot(),
        new ParallelRaceGroup(
            new AutoIntakeFeed(feeder, rollers, intakeRollers, shooter.beam, shooter.beamOne),
            Overbump_Depot.getNext()),
        Shoot());
  }

  public Command Test() {
    currentSequence = TestPath;
    logPoints("Test", currentSequence);
    return new SequentialCommandGroup(
        resetPose(TestPath),
        new ParallelRaceGroup(
            new AutoIntakeFeed(feeder, rollers, intakeRollers, shooter.beam, shooter.beamOne),
            TestPath.getNext()));
  }

  public static void logPoints(String name, BLinePathSequence sequence) {
    Logger.log(name, getPointsFromSequence(sequence));
  }

  private static Pose2d[] getPointsFromSequence(BLinePathSequence sequence) {
    ArrayList<Path> paths = new ArrayList<>();
    ArrayList<Pose2d> points = new ArrayList<>();
    for (int i = 0; i < sequence.getSize(); i++) {
      sequence.getPath(i).ifPresent(path -> paths.add(path));
    }

    paths.forEach(
        path -> {
          points.add(path.getStartPose());
          getPoseFromElement(path.getElement(path.getPathElements().size() - 1))
              .ifPresent(pose -> points.add(pose));
        });

    return points.toArray(Pose2d[]::new);
  }

  private static Optional<Pose2d> getPoseFromElement(PathElement element) {
    if (element instanceof Waypoint) {
      Waypoint waypoint = (Waypoint) element;
      Translation2d translation = waypoint.translationTarget().translation();
      Rotation2d rotation = waypoint.rotationTarget().rotation();
      return Optional.of(new Pose2d(translation, rotation));
    } else {
      return Optional.empty();
    }
  }
}
