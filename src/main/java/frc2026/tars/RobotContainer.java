package frc2026.tars;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.team5000.HubTracker;
import com.teamscreamrobotics.dashboard.MechanismVisualizer;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2026.tars.autonomous.Routines;
import frc2026.tars.commands.IntakeFeed;
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
import frc2026.tars.subsystems.leds.LED;
import frc2026.tars.subsystems.shooter.Shooter;
import frc2026.tars.subsystems.shooter.Shooter.ShooterState;
import frc2026.tars.subsystems.shooter.feeder.Feeder;
import frc2026.tars.subsystems.shooter.feeder.FeederConstants;
import frc2026.tars.subsystems.shooter.flywheel.Flywheel;
import frc2026.tars.subsystems.shooter.flywheel.FlywheelConstants;
import frc2026.tars.subsystems.shooter.hood.Hood;
import frc2026.tars.subsystems.shooter.hood.HoodConstants;
import frc2026.tars.subsystems.shooter.rollers.Rollers;
import frc2026.tars.subsystems.shooter.rollers.RollersConstants;
import frc2026.tars.subsystems.vision.VisionManager;
import lombok.Getter;

public class RobotContainer {

  public record Subsystems(
      Drivetrain drivetrain,
      IntakeWrist intakeWrist,
      IntakeRollers intakeRollers,
      Feeder feeder,
      Rollers rollers,
      Hood hood,
      Flywheel flywheel,
      LED led) {}

  private final LED led = new LED();

  private final IntakeWrist intakeWrist = new IntakeWrist(IntakeConstants.WRIST_CONFIG);
  private final IntakeRollers intakeRollers = new IntakeRollers(IntakeConstants.ROLLERS_CONFIG);
  private final Drivetrain drivetrain = TunerConstants.drivetrain;

  private final Hood hood = new Hood(HoodConstants.HOOD_CONFIG);
  private final Flywheel flywheel = new Flywheel(FlywheelConstants.FLYWHEEL_CONFIG);
  private final Rollers rollers = new Rollers(RollersConstants.ROLLERS_CONFIG);
  private final Feeder feeder = new Feeder(FeederConstants.FEEDER_CONFIG);

  @Getter
  private final Subsystems subsystems =
      new Subsystems(drivetrain, intakeWrist, intakeRollers, feeder, rollers, hood, flywheel, led);

  @Getter private final RobotState robotState = new RobotState(subsystems);

  Field2d field = new Field2d();

  private final Shooter shooter =
      new Shooter(
          flywheel,
          hood,
          intakeWrist,
          intakeRollers,
          feeder,
          rollers,
          led,
          drivetrain,
          getRobotState());

  private final VisionManager visionManager = new VisionManager(drivetrain);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveDriveBrake();

  private final MechanismVisualizer mechVisualizer =
      new MechanismVisualizer(
          SimConstants.MEASURED_MECHANISM,
          SimConstants.SETPOINT_MECHANISM,
          RobotContainer::telemeterizeMechanisms,
          intakeWrist.intakeMech);

  private final Routines routines = new Routines(drivetrain, shooter, robotState, this);

  public RobotContainer() {
    configureBindings();
    configureManualOverrides();
    configureDefaultCommands();

    SmartDashboard.putNumber("test", 1);

    mechVisualizer.setEnabled(true);
  }

  private void configureBindings() {

    Controlboard.intake()
        .whileTrue(intakeRollers.applyGoalCommand(IntakeRollersGoal.INTAKE))
        .onFalse(new IntakeFeed(feeder, rollers, shooter.beam, shooter.beamOne));

    Controlboard.shoot()
        .whileTrue(
            Commands.parallel(
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
                intakeWrist
                    .compress(() -> robotState.atTargetAngle())
                    .onlyWhile(() -> shooter.getState() == ShooterState.SHOOTING),
                intakeRollers.applyGoalCommand(IntakeRollersGoal.COMPRESS)));

    Controlboard.moveIntakeWrist()
        .whileTrue(
            Commands.runEnd(
                () -> intakeWrist.applyGoal(IntakeWristGoal.STOW),
                () -> intakeWrist.applyGoal(IntakeWristGoal.EXTENDED),
                intakeWrist));

    Controlboard.lockSwerve().whileTrue(drivetrain.applyRequest(() -> brake));

    Controlboard.rotate90Degrees()
        .whileTrue(
            drivetrain
                .applyRequest(
                    () ->
                        drivetrain
                            .getHelper()
                            .getFacingAngleProfiled(
                                Controlboard.getTranslation(),
                                Rotation2d.fromDegrees(90),
                                DrivetrainConstants.headingControllerProfiled))
                .beforeStarting(() -> drivetrain.resetHeadingController()));

    Controlboard.rotateNegative90Degrees()
        .whileTrue(
            drivetrain
                .applyRequest(
                    () ->
                        drivetrain
                            .getHelper()
                            .getFacingAngleProfiled(
                                Controlboard.getTranslation(),
                                Rotation2d.fromDegrees(-90),
                                DrivetrainConstants.headingControllerProfiled))
                .beforeStarting(() -> drivetrain.resetHeadingController()));
    Controlboard.rotate0Degrees()
        .whileTrue(
            drivetrain
                .applyRequest(
                    () ->
                        drivetrain
                            .getHelper()
                            .getFacingAngleProfiled(
                                Controlboard.getTranslation(),
                                Rotation2d.fromDegrees(0),
                                DrivetrainConstants.headingControllerProfiled))
                .beforeStarting(() -> drivetrain.resetHeadingController()));
    Controlboard.rotate180Degrees()
        .whileTrue(
            drivetrain
                .applyRequest(
                    () ->
                        drivetrain
                            .getHelper()
                            .getFacingAngleProfiled(
                                Controlboard.getTranslation(),
                                Rotation2d.fromDegrees(180),
                                DrivetrainConstants.headingControllerProfiled))
                .beforeStarting(() -> drivetrain.resetHeadingController()));

    Controlboard.runBackHopper()
        .onTrue(
            intakeRollers
                .applyVoltageCommand(() -> -12.0)
                .alongWith(rollers.applyVoltageCommand(() -> -12.0))
                .alongWith(feeder.applyVoltageCommand(() -> -12.0)))
        .onFalse(
            intakeRollers
                .applyVoltageCommand(() -> 0.0)
                .alongWith(rollers.applyVoltageCommand(() -> 0.0))
                .alongWith(feeder.applyVoltageCommand(() -> 0.0)));
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
                                    .times(RobotState.getSpeedLimit().getAsDouble()),
                                Controlboard.getRotation().getAsDouble())
                        : drivetrain
                            .getHelper()
                            .getRobotCentric(
                                Controlboard.getTranslation()
                                    .times(RobotState.getSpeedLimit().getAsDouble()),
                                Controlboard.getRotation().getAsDouble()))
            .beforeStarting(() -> drivetrain.getHelper().setLastAngle(drivetrain.getHeading()))
            .withName("Drivetrain: Default command"));

    shooter.setDefaultCommand(shooter.defaultCommand());

    led.setDefaultCommand(
        led.run(
                () -> {
                  if (DriverStation.isDisabled()) {
                    led.larson(
                        () ->
                            (AllianceFlipUtil.shouldFlip().getAsBoolean()
                                ? Color.kGreen
                                : Color.kBlue),
                        1.25);
                  } else {
                    robotState.flashLEDS();
                  }
                })
            .ignoringDisable(true));
  }

  private void configureManualOverrides() {
    Controlboard.runBackFlywheel().whileTrue(flywheel.applyVoltageCommand(() -> -1.0));

    Controlboard.resetFieldCentric()
        .onTrue(Commands.runOnce(() -> drivetrain.resetRotation(AllianceFlipUtil.getFwdHeading())));

    Controlboard.zeroIntake()
        .whileTrue(
            Commands.runOnce(() -> intakeWrist.resetPosition(0.0), intakeWrist)
                .andThen(() -> Dashboard.zeroIntake.set(false))
                .ignoringDisable(true));

    Controlboard.zeroHood().whileTrue(hood.zero().andThen(() -> Dashboard.zeroHood.set(false)));

    Controlboard.runBackIntake()
        .whileTrue(
            (intakeRollers.applyVoltageCommand(() -> -2.0))
                .andThen(() -> Dashboard.runBackIntake.set(false)));

    Controlboard.resetManuals()
        .whileTrue(
            (Commands.runOnce(() -> Dashboard.resetManuals())
                    .andThen(() -> Dashboard.resetManuals.set(false)))
                .ignoringDisable(true));

    Controlboard.getManualMode()
        .whileTrue(
            Commands.parallel(
                    Commands.run(
                        () ->
                            hood.moveToAngle(
                                Rotation2d.fromDegrees(Dashboard.manualHoodAngle.get())),
                        hood),
                    Commands.run(
                        () ->
                            flywheel.setTargetVelocityTorqueCurrent(
                                Dashboard.manualFlywheelVelocity.get(), 0.0),
                        flywheel),
                    Commands.run(
                        () ->
                            intakeWrist.moveToAngle(
                                Rotation2d.fromDegrees(Dashboard.manualIntakeWrist.get())),
                        intakeWrist),
                    Commands.run(
                        () -> intakeRollers.setVoltage(Dashboard.manualIntakeRollers.get()),
                        intakeRollers),
                    Commands.run(
                        () -> rollers.setVoltage(Dashboard.manualFloorRollers.get()), rollers),
                    Commands.run(() -> feeder.setVoltage(Dashboard.manualFeeder.get()), feeder))
                .ignoringDisable(true));

    Controlboard.bumperShot()
        .whileTrue(
            Commands.parallel(
                    feeder.applyVoltageCommand(() -> 12.0),
                    rollers.applyVoltageCommand(() -> 12.0),
                    Commands.run(() -> hood.moveToAngle(Rotation2d.fromDegrees(12.5))),
                    intakeWrist.compress(() -> true))
                .onlyIf(() -> flywheel.atVel()));

    Controlboard.bumperShot()
        .whileTrue(flywheel.run(() -> flywheel.setTargetVelocityTorqueCurrent(33.6, 0.0)));
  }

  public Command getAutonomousCommand() {
    return routines.getRoutineChooser().getSelected();
  }

  public static void telemeterizeMechanisms(Mechanism2d measured, Mechanism2d setpoint) {
    Logger.log("RobotState/Mechanisms/Measured", measured);
    Logger.log("RobotState/Mechanisms/Setpoint", setpoint);
  }

  public void periodic() {
    visionManager.periodic();
    robotState.logArea();

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    HubTracker.timeRemainingInCurrentShift()
        .ifPresentOrElse(
            t -> SmartDashboard.putNumber("Time in shift", t.abs(Seconds)),
            () -> SmartDashboard.putNumber("Time in shift", -1.0));

    SmartDashboard.putBoolean("Is Active", HubTracker.isActive());
  }
}
