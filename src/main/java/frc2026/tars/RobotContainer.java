package frc2026.tars;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.teamscreamrobotics.dashboard.MechanismVisualizer;
import com.teamscreamrobotics.gameutil.FieldConstants;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc2026.tars.subsystems.indexer.Feeder;
import frc2026.tars.subsystems.indexer.IndexerConstants;
import frc2026.tars.subsystems.indexer.Spindexer;
import frc2026.tars.subsystems.indexer.Feeder.FeederGoal;
import frc2026.tars.subsystems.indexer.Spindexer.SpindexerGoal;
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
import frc2026.tars.subsystems.shooter.turret.Turret;
import frc2026.tars.subsystems.shooter.turret.TurretConstants;
import frc2026.tars.subsystems.vision.VisionManager;
import lombok.Getter;

public class RobotContainer {

  public record Subsystems(
      Drivetrain drivetrain,
      IntakeWrist intakeWrist,
      Shooter shooter,
      Turret turret,
      Hood hood,
      Flywheel flywheel,
      Spindexer spindexer,
      Feeder feeder) {}

  private final IntakeWrist intakeWrist = new IntakeWrist(IntakeConstants.INTAKE_WRIST_CONFIG);
  private final IntakeRollers intakeRollers = new IntakeRollers(IntakeConstants.ROLLERS_CONFIG);
  private final Drivetrain drivetrain = TunerConstants.drivetrain;

  private final Shooter shooter = new Shooter(null, null);
  private final Turret turret = new Turret(TurretConstants.TURRET_CONFIG);
  private final Hood hood = new Hood(HoodConstants.HOOD_CONFIG);
  private final Flywheel flywheel = new Flywheel(FlywheelConstants.FLYWHEEL_CONFIG);
  private final Climber climber = new Climber(HoodConstants.HOOD_CONFIG);

  private final Spindexer spindexer = new Spindexer(IndexerConstants.SPINDEXER_CONFIG);
  private final Feeder feeder = new Feeder(IndexerConstants.FEEDER_CONFIG);

  private final VisionManager visionManager = new VisionManager(drivetrain, turret);

  @Getter
  private final Subsystems subsystems =
      new Subsystems(drivetrain, intakeWrist, shooter, turret, hood, flywheel, spindexer, feeder);

  @Getter private final RobotState robotState = new RobotState(subsystems);

  private final MechanismVisualizer mechVisualizer =
      new MechanismVisualizer(
          SimConstants.MEASURED_MECHANISM,
          SimConstants.SETPOINT_MECHANISM,
          RobotContainer::telemeterizeMechanisms,
          intakeWrist.intakeMech);

          public Translation2d getFerryZone() {
    if (drivetrain.getEstimatedPose().getY() >= FieldConstants.fieldWidth / 2.0) {
      return AllianceFlipUtil.get(
          FieldConstants.AllianceZones.leftAllianceZone,
          FieldConstants.AllianceZones.oppLeftAllianceZone);
    } else {
      return AllianceFlipUtil.get(
          FieldConstants.AllianceZones.rightAllianceZone,
          FieldConstants.AllianceZones.oppRightAllianceZone);
    }
  }

  public BooleanSupplier inAllianceZone() {
    if (drivetrain.getEstimatedPose().getX()
        >= AllianceFlipUtil.get(
            FieldConstants.fieldLength / 4, (FieldConstants.fieldLength * 3) / 4)) {
      return () -> true;
    } else {
      return () -> false;
    }
  }

  // public Rotation2d getCrossedReferencedAngle() {
  //   double visionAngle =
  // Units.radiansToDegrees(vision.getRotation(Length.fromInches(MaxAngularRate).getMeters()));
  //   double
  // }

  public Command aimCommand() {
    return turret.aimOnTheFlyPosition(
        () ->
            (inAllianceZone().getAsBoolean() == false
                ? getFerryZone()
                : AllianceFlipUtil.get(
                    FieldConstants.Hub.hubCenter, FieldConstants.Hub.oppHubCenter)),
        () -> drivetrain.getEstimatedPose(),
        () ->
            new ChassisSpeeds(
                drivetrain.getState().Speeds.vxMetersPerSecond,
                drivetrain.getState().Speeds.vyMetersPerSecond,
                drivetrain.getState().Speeds.omegaRadiansPerSecond));
  }

  public RobotContainer() {
    configureBindings();
    configureManualOverrides();
    configureDefaultCommands();
    configureAutoCommands();

    SmartDashboard.putNumber("test", 1);

    mechVisualizer.setEnabled(true);
  }

  private void configureBindings() {
    Controlboard.makeThingWork().whileTrue(Commands.parallel(spindexer.applyGoalCommand(SpindexerGoal.RUN), feeder.applyGoalCommand(FeederGoal.RUN))).whileFalse(Commands.parallel(spindexer.applyGoalCommand(SpindexerGoal.STOP), feeder.applyGoalCommand(FeederGoal.STOP)));

    Controlboard.intake()
        .onTrue(
            new SequentialCommandGroup(
                    intakeRollers
                        .applyGoalCommand(IntakeRollers.IntakeRollersGoal.INTAKE)
                        .alongWith(
                            intakeWrist.applyGoalCommand(IntakeWrist.IntakeWristGoal.EXTENDED)))
                .withName("Intaking"))
        .onFalse(
            new SequentialCommandGroup(
                    intakeRollers
                        .applyGoalCommand(IntakeRollers.IntakeRollersGoal.STOP)
                        .alongWith(intakeWrist.applyGoalCommand(IntakeWrist.IntakeWristGoal.STOW)))
                .withName("Stowed"));
  }

  private void configureDefaultCommands() {

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
                                -Controlboard.getRotation().getAsDouble())
                        : drivetrain
                            .getHelper()
                            .getRobotCentric(
                                Controlboard.getTranslation()
                                    .get()
                                    .times(RobotState.getSpeedLimit().getAsDouble()),
                                Controlboard.getRotation().getAsDouble()))
            .beforeStarting(() -> drivetrain.getHelper().setLastAngle(drivetrain.getHeading()))
            .withName("Drivetrain: Default command"));

            turret.setDefaultCommand(aimCommand());
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
        .whileTrue(
            drivetrain
                .runOnce(() -> drivetrain.seedFieldCentric())
                .andThen(() -> Dashboard.zeroIntake.set(false)));

    Controlboard.zeroIntake()
        .whileTrue(intakeWrist.zero().andThen(() -> Dashboard.zeroIntake.set(false)));

    Controlboard.zeroClimber()
        .whileTrue(climber.zero().andThen(() -> Dashboard.zeroClimber.set(false)));

    Controlboard.zeroHood().whileTrue(hood.zero().andThen(() -> Dashboard.zeroHood.set(false)));
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
    robotState.logArea();
  }
}
