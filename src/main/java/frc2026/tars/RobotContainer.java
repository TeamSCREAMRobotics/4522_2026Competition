package frc2026.tars;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.teamscreamrobotics.dashboard.MechanismVisualizer;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2026.tars.constants.SimConstants;
import frc2026.tars.controlboard.Controlboard;
import frc2026.tars.controlboard.Dashboard;
import frc2026.tars.subsystems.drivetrain.Drivetrain;
import frc2026.tars.subsystems.drivetrain.DrivetrainConstants;
import frc2026.tars.subsystems.drivetrain.generated.TunerConstants;
import frc2026.tars.subsystems.intake.IntakeConstants;
import frc2026.tars.subsystems.intake.IntakeRollers;
import frc2026.tars.subsystems.intake.IntakeRollers.IntakeRollersGoal;
import frc2026.tars.subsystems.intake.IntakeWrist;
import frc2026.tars.subsystems.intake.IntakeWrist.IntakeWristGoal;
import frc2026.tars.subsystems.shooter.Shooter;
import frc2026.tars.subsystems.shooter.dyerotor.Dyerotor;
import frc2026.tars.subsystems.shooter.dyerotor.DyerotorConstants;
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
      Turret turret,
      Hood hood,
      Flywheel flywheel) {}

  private final IntakeWrist intakeWrist = new IntakeWrist(IntakeConstants.WRIST_CONFIG);
  private final IntakeRollers intakeRollers = new IntakeRollers(IntakeConstants.ROLLERS_CONFIG);
  private final Drivetrain drivetrain = TunerConstants.drivetrain;

  private final Turret turret = new Turret(TurretConstants.TURRET_CONFIG);
  private final Hood hood = new Hood(HoodConstants.HOOD_CONFIG);
  private final Flywheel flywheel = new Flywheel(FlywheelConstants.FLYWHEEL_CONFIG);
  private final Dyerotor spindexer = new Dyerotor(DyerotorConstants.DYEROTOR_CONFIG);

  @Getter
  private final Subsystems subsystems =
      new Subsystems(drivetrain, intakeWrist, turret, hood, flywheel);

  @Getter private final RobotState robotState = new RobotState(subsystems);

  private final Shooter shooter =
      new Shooter(flywheel, hood, turret, spindexer, intakeWrist, drivetrain, getRobotState());

  private final VisionManager visionManager = new VisionManager(drivetrain, turret);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveDriveBrake();

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

  private final SendableChooser<Command> auto;

  public RobotContainer() {
    configureBindings();
    configureManualOverrides();
    configureDefaultCommands();
    configureAutoCommands();

    SmartDashboard.putNumber("test", 1);

    auto = AutoBuilder.buildAutoChooser();
    auto.setDefaultOption("Do Nothing", new PathPlannerAuto("command"));
    SmartDashboard.putData(auto);

    mechVisualizer.setEnabled(true);
  }

  private void configureBindings() {

    Controlboard.intake()
        .onTrue(
            new SequentialCommandGroup(
                    intakeRollers.applyGoalCommand(IntakeRollers.IntakeRollersGoal.INTAKE))
                .withName("Intake Running"))
        .onFalse(
            new SequentialCommandGroup(
                    intakeRollers.applyGoalCommand(IntakeRollers.IntakeRollersGoal.STOP))
                .withName("Intake Stopped"));

    Controlboard.moveIntakeWrist()
        .toggleOnTrue(intakeWrist.applyGoalCommand(IntakeWristGoal.STOW))
        .toggleOnFalse(intakeWrist.applyGoalCommand(IntakeWristGoal.EXTENDED));

    Controlboard.lockSwerve().whileTrue(drivetrain.applyRequest(() -> brake));

    Controlboard.rotate90Degres()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drivetrain
                        .getHelper()
                        .getFacingAngleProfiled(
                            Controlboard.getTranslation().get(),
                            Rotation2d.fromDegrees(90),
                            DrivetrainConstants.headingControllerProfiled)));
    Controlboard.rotateNegative90Degrees()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drivetrain
                        .getHelper()
                        .getFacingAngleProfiled(
                            Controlboard.getTranslation().get(),
                            Rotation2d.fromDegrees(-90),
                            DrivetrainConstants.headingControllerProfiled)));

    Controlboard.hailMaryMode()
        .whileTrue(
            new SequentialCommandGroup(turret.moveToAngleCommandRR(Rotation2d.fromDegrees(0.0)))
                .alongWith(hood.moveToAngleCommand(Rotation2d.fromDegrees(0.0)))
                .alongWith(flywheel.setTargetVelocityTorqueCurrentCommand(40.5, 0.0)));
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
        "Run Intake", intakeRollers.applyGoalCommand(IntakeRollersGoal.INTAKE).withTimeout(2.0));

    NamedCommands.registerCommand(
        "Intake In",
        new SequentialCommandGroup(
                intakeWrist
                    .applyGoalCommand(IntakeWristGoal.STOW)
                    .alongWith(intakeRollers.applyGoalCommand(IntakeRollersGoal.STOP)))
            .withName("Auto Intake In"));

    NamedCommands.registerCommand(
        "Stop Intakeing", intakeRollers.applyGoalCommand(IntakeRollersGoal.STOP).withTimeout(0.1));

    NamedCommands.registerCommand("Short Shoot", shooter.autoShoot(5.0));
  }

  private void configureManualOverrides() {
    Controlboard.resetFieldCentric()
        .onTrue(Commands.runOnce(() -> drivetrain.resetRotation(AllianceFlipUtil.getFwdHeading())));

    Controlboard.zeroIntake()
        .whileTrue(
            Commands.runOnce(() -> intakeWrist.resetPosition(0.0), intakeWrist)
                .andThen(() -> Dashboard.zeroIntake.set(false))
                .ignoringDisable(true));

    Controlboard.zeroHood().whileTrue(hood.zero().andThen(() -> Dashboard.zeroHood.set(false)));

    Controlboard.zeroTurret()
        .onTrue(
            turret.setZero().andThen(() -> Dashboard.zeroTurret.set(false)).ignoringDisable(true));

    Controlboard.getManualMode()
        .whileTrue(
            Commands.parallel(
                    turret.moveToAngleCommandFR(
                        () -> Rotation2d.fromDegrees(Dashboard.manualTurretAngle.get()),
                        () -> drivetrain.getEstimatedPose().getRotation()),
                    hood.moveToAngleCommand(
                        Rotation2d.fromDegrees(Dashboard.manualHoodAngle.get())),
                    Commands.run(
                        () -> flywheel.setVoltage(Dashboard.manualFlywheelVelocity.get()),
                        flywheel))
                .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return auto.getSelected();
  }

  public static void telemeterizeMechanisms(Mechanism2d measured, Mechanism2d setpoint) {
    Logger.log("RobotState/Mechanisms/Measured", measured);
    Logger.log("RobotState/Mechanisms/Setpoint", setpoint);
  }

  public void periodic() {
    visionManager.periodic();
    // robotState.logArea();

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
                    - turret.getAngle().getRadians())));

    // Logger.log("Subsystems/Turret/Angle Setpoint",
    // ScreamMath.calculateAngleToPoint(drivetrain.getEstimatedPose().getTranslation(),
    // FieldConstants.Hub.hubCenter).getDegrees());
  }
}
