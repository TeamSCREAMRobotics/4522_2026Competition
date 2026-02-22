package frc2026.tars;

import com.pathplanner.lib.auto.NamedCommands;
import com.teamscreamrobotics.dashboard.MechanismVisualizer;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2026.tars.constants.SimConstants;
import frc2026.tars.controlboard.Controlboard;
import frc2026.tars.controlboard.Dashboard;
import frc2026.tars.subsystems.climber.Climber;
import frc2026.tars.subsystems.drivetrain.Drivetrain;
import frc2026.tars.subsystems.drivetrain.generated.TunerConstants;
import frc2026.tars.subsystems.intake.IntakeConstants;
import frc2026.tars.subsystems.intake.IntakeRollers;
import frc2026.tars.subsystems.intake.IntakeRollers.IntakeRollersGoal;
import frc2026.tars.subsystems.intake.IntakeWrist;
import frc2026.tars.subsystems.intake.IntakeWrist.IntakeWristGoal;
import frc2026.tars.subsystems.shooter.Shooter;
import frc2026.tars.subsystems.shooter.flywheel.Flywheel;
import frc2026.tars.subsystems.shooter.flywheel.FlywheelConstants;
import frc2026.tars.subsystems.shooter.hood.Hood;
import frc2026.tars.subsystems.shooter.hood.HoodConstants;
import frc2026.tars.subsystems.shooter.indexer.Feeder;
import frc2026.tars.subsystems.shooter.indexer.Feeder.FeederGoal;
import frc2026.tars.subsystems.shooter.indexer.IndexerConstants;
import frc2026.tars.subsystems.shooter.indexer.Spindexer;
import frc2026.tars.subsystems.shooter.indexer.Spindexer.SpindexerGoal;
import frc2026.tars.subsystems.shooter.turret.Turret;
import frc2026.tars.subsystems.shooter.turret.TurretConstants;
import frc2026.tars.subsystems.vision.VisionManager;
import lombok.Getter;

public class RobotContainer {

  public record Subsystems(
      Drivetrain drivetrain,
      IntakeWrist intakeWrist,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      Feeder feeder) {}

  private final IntakeWrist intakeWrist = new IntakeWrist(IntakeConstants.INTAKE_WRIST_CONFIG);
  private final IntakeRollers intakeRollers = new IntakeRollers(IntakeConstants.ROLLERS_CONFIG);
  private final Drivetrain drivetrain = TunerConstants.drivetrain;

  private final Turret turret = new Turret(TurretConstants.TURRET_CONFIG);
  private final Hood hood = new Hood(HoodConstants.HOOD_CONFIG);
  private final Flywheel flywheel = new Flywheel(FlywheelConstants.FLYWHEEL_CONFIG);
  private final Climber climber = new Climber(HoodConstants.HOOD_CONFIG);
  private final Spindexer spindexer = new Spindexer(IndexerConstants.SPINDEXER_CONFIG);
  private final Feeder feeder = new Feeder(IndexerConstants.FEEDER_CONFIG);

  @Getter
  private final Subsystems subsystems =
      new Subsystems(drivetrain, intakeWrist, turret, hood, flywheel, feeder);

  @Getter private final RobotState robotState = new RobotState(subsystems);

  private final Shooter shooter =
      new Shooter(flywheel, hood, turret, spindexer, feeder, drivetrain, getRobotState());

  private final VisionManager visionManager = new VisionManager(drivetrain, turret);

  private final MechanismVisualizer mechVisualizer =
      new MechanismVisualizer(
          SimConstants.MEASURED_MECHANISM,
          SimConstants.SETPOINT_MECHANISM,
          RobotContainer::telemeterizeMechanisms,
          intakeWrist.intakeMech);

  // public Rotation2d getCrossedReferencedAngle() {
  //   double visionAngle =
  // Units.radiansToDegrees(vision.getRotation(Length.fromInches(MaxAngularRate).getMeters()));
  //   double
  // }

  // public Command aimCommand() {
  //   return turret.aimOnTheFlyPosition(
  //       () ->
  //           (inAllianceZone().getAsBoolean() == false
  //               ? getFerryZone()
  //               : AllianceFlipUtil.get(
  //                   FieldConstants.Hub.hubCenter, FieldConstants.Hub.oppHubCenter)),
  //       () -> drivetrain.getEstimatedPose(),
  //       () ->
  //           new ChassisSpeeds(
  //               drivetrain.getState().Speeds.vxMetersPerSecond,
  //               drivetrain.getState().Speeds.vyMetersPerSecond,
  //               drivetrain.getState().Speeds.omegaRadiansPerSecond));
  // }

  public RobotContainer() {
    configureBindings();
    configureManualOverrides();
    configureDefaultCommands();
    configureAutoCommands();

    SmartDashboard.putNumber("test", 1);

    mechVisualizer.setEnabled(true);
  }

  private void configureBindings() {
    Controlboard.makeThingWork()
        .whileTrue(
            Commands.parallel(
                spindexer.applyGoalCommand(SpindexerGoal.RUN),
                feeder.applyGoalCommand(FeederGoal.RUN)))
        .whileFalse(
            Commands.parallel(
                spindexer.applyGoalCommand(SpindexerGoal.STOP),
                feeder.applyGoalCommand(FeederGoal.STOP)));

    Controlboard.intake()
        .onTrue(
            new SequentialCommandGroup(
                    intakeRollers.applyGoalCommand(IntakeRollers.IntakeRollersGoal.INTAKE))
                .withName("Intakeing"))
        .onFalse(
            new SequentialCommandGroup(
                    intakeRollers.applyGoalCommand(IntakeRollers.IntakeRollersGoal.STOP))
                .withName("Not Intake"));
  }

  private void configureDefaultCommands() {

    /* turret.setDefaultCommand(
    turret.aimOnTheFlyPosition(
        () -> FieldConstants.Hub.oppHubCenter,
        () -> drivetrain.getState().Pose,
        () -> drivetrain.getState().Speeds)); */

    drivetrain.setDefaultCommand(
        drivetrain
            .applyRequest(
                () ->
                    Controlboard.getFieldCentric().getAsBoolean()
                        ? drivetrain
                            .getHelper()
                            .getFieldCentric(
                                Controlboard.getTranslation()
                                    .get()
                                    .times(RobotState.getSpeedLimit().getAsDouble()),
                                Controlboard.getRotation().getAsDouble())
                        : drivetrain
                            .getHelper()
                            .getRobotCentric(
                                Controlboard.getTranslation()
                                    .get()
                                    .times(RobotState.getSpeedLimit().getAsDouble()),
                                Controlboard.getRotation().getAsDouble()))
            .beforeStarting(() -> drivetrain.getHelper().setLastAngle(drivetrain.getHeading()))
            .withName("Drivetrain: Default command"));

    shooter.setDefaultCommand(shooter.defaultCommand());
  }

  private void configureAutoCommands() {

    NamedCommands.registerCommand(
        "Intake Out",
        new SequentialCommandGroup(
                intakeWrist
                    .applyGoalCommand(IntakeWristGoal.STOW)
                    .alongWith(intakeRollers.applyGoalCommand(IntakeRollersGoal.INTAKE)))
            .withName("Auto Intake Out"));

    NamedCommands.registerCommand(
        "Intake In",
        new SequentialCommandGroup(
                intakeWrist
                    .applyGoalCommand(IntakeWristGoal.STOW)
                    .alongWith(intakeRollers.applyGoalCommand(IntakeRollersGoal.INTAKE)))
            .withName("Auto Intake Out"));
  }

  private void configureManualOverrides() {
    Controlboard.resetFieldCentric()
        .onTrue(Commands.runOnce(() -> drivetrain.resetRotation(AllianceFlipUtil.getFwdHeading())));

    Controlboard.zeroIntake()
        .whileTrue(intakeWrist.zero().andThen(() -> Dashboard.zeroIntake.set(false)));

    Controlboard.zeroClimber()
        .whileTrue(climber.zero().andThen(() -> Dashboard.zeroClimber.set(false)));

    Controlboard.zeroHood().whileTrue(hood.zero().andThen(() -> Dashboard.zeroHood.set(false)));

    Controlboard.zeroTurret()
        .whileTrue(turret.setZero().andThen(() -> Dashboard.zeroTurret.set(false)));

    Controlboard.getManualMode()
        .whileTrue(
            Commands.parallel(
                    turret.moveToAngleCommandFR(
                        () -> Rotation2d.fromDegrees(Dashboard.manualTurretAngle.get()),
                        () -> drivetrain.getEstimatedPose().getRotation()))
                .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static void telemeterizeMechanisms(Mechanism2d measured, Mechanism2d setpoint) {
    Logger.log("RobotState/Mechanisms/Measured", measured);
    Logger.log("RobotState/Mechanisms/Setpoint", setpoint);
  }

  public void periodic() {
    visionManager.periodic();
    // robotState.logArea();

    /*
    Tell this to jackson dummy future me, turret looked correct in sim, so that means that our values or calculations for CRT are prob wrong.
    Things that we need to check:
    - Are we using the correct gear ratio for the turret?
    - is the calculation for CRT correct?
    - Is zeroing right? (I guess this is connected to CRT)
    - Pose?
    - Pigeon bad?
    - Some weird issue with the DogLog stuff?
    - Rio issue?
    - Sensor Direction Value?
    */

    Logger.log(
        "Turret Pose",
        new Pose3d(
            drivetrain.getEstimatedPose().getX(),
            drivetrain.getEstimatedPose().getY(),
            0.5,
            new Rotation3d(
                0,
                0,
                drivetrain.getEstimatedPose().getRotation().getRadians()
                    + turret.getAngle().getRadians())));

    // Logger.log("Subsystems/Turret/Angle Setpoint",
    // ScreamMath.calculateAngleToPoint(drivetrain.getEstimatedPose().getTranslation(),
    // FieldConstants.Hub.hubCenter).getDegrees());
  }
}
