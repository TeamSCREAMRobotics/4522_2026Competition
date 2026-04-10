package frc2026.tars;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.teamscreamrobotics.dashboard.MechanismVisualizer;
import com.teamscreamrobotics.util.AllianceFlipUtil;
import com.teamscreamrobotics.util.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
      Drivetrain drivetrain, IntakeWrist intakeWrist, Hood hood, Flywheel flywheel, LED led) {}

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
      new Subsystems(drivetrain, intakeWrist, hood, flywheel, led);

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
          // hopper,
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

  private final SendableChooser<Command> auto;

  private Debouncer stupidahhenginnerdebouncher = new Debouncer(0.35, DebounceType.kRising);

  public RobotContainer() {
    configureBindings();
    configureManualOverrides();
    configureDefaultCommands();
    configureAutoCommands();

    SmartDashboard.putNumber("test", 1);

    auto = AutoBuilder.buildAutoChooser();
    auto.setDefaultOption("Do Nothing", null);
    SmartDashboard.putData(auto);
    mechVisualizer.setEnabled(true);
  }

  private void configureBindings() {

    Controlboard.intake()
        .whileTrue(
            new SequentialCommandGroup(
                    intakeRollers.applyGoalCommand(IntakeRollers.IntakeRollersGoal.INTAKE))
                .withName("Intake Running"));

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
                intakeWrist.compress(
                        () -> robotState.atTargetAngle()),
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

    Controlboard.makeThingWork()
        .whileTrue(
            new SequentialCommandGroup(
                Commands.parallel(
                        rollers.applyVoltageCommand(() -> 1.0),
                        feeder.applyVoltageCommand(() -> 2.0))
                    .withDeadline(
                        new WaitUntilCommand(
                            () ->
                                stupidahhenginnerdebouncher.calculate(
                                    shooter.beam.getIsDetected().getValue()))),
                Commands.parallel(
                    rollers.applyVoltageCommand(() -> 0.0),
                    feeder.applyVoltageCommand(() -> 0.0))));

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

    // Controlboard.runBackHopper().whileTrue(Commands.parallel(intakeRollers.applyVoltageCommand(()
    // -> -3.0), rollers.applyVoltageCommand(() -> -3.0), feeder.applyVoltageCommand(() -> -3.0)));

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

  private void configureAutoCommands() {

    NamedCommands.registerCommand(
        "Run Intake", intakeRollers.applyGoalCommand(IntakeRollersGoal.AUTOINTAKE));

    NamedCommands.registerCommand(
        "Stop Intake", intakeRollers.applyGoalCommand(IntakeRollersGoal.STOP));

    NamedCommands.registerCommand(
        "BlipFuel",
        Commands.parallel(
                rollers.applyVoltageCommand(() -> 1.0), feeder.applyVoltageCommand(() -> 2.0))
            .until(() -> shooter.beam.getIsDetected().getValue())
            .andThen(
                Commands.parallel(
                    rollers.applyVoltageCommand(() -> 1.0),
                    feeder.applyVoltageCommand(() -> 2.0))));

    NamedCommands.registerCommand(
        "Intake In",
        new SequentialCommandGroup(
                intakeWrist
                    .applyGoalCommand(IntakeWristGoal.STOW)
                    .alongWith(intakeRollers.applyGoalCommand(IntakeRollersGoal.STOP)))
            .withName("Auto Intake In"));

    NamedCommands.registerCommand(
        "Stop Intakeing", intakeRollers.applyGoalCommand(IntakeRollersGoal.STOP).withTimeout(0.1));

    NamedCommands.registerCommand(
        "Aim at Hub",
        drivetrain
            .applyRequest(
                () ->
                    drivetrain
                        .getHelper()
                        .getFacingAngleProfiled(
                            Controlboard.getTranslation(),
                            robotState.getDrivetrainTarget(),
                            DrivetrainConstants.headingControllerProfiled))
            .beforeStarting(() -> drivetrain.resetHeadingController()));

    NamedCommands.registerCommand("Shoot", shooter.autoShoot());

    NamedCommands.registerCommand(
        "IntakeWristOut",
        intakeWrist.runOnce(() -> intakeWrist.applyGoal(IntakeWristGoal.EXTENDED)));

    NamedCommands.registerCommand(
        "Run Kickers",
        intakeRollers
            .applyGoalCommand(IntakeRollersGoal.INTAKE)
            .withTimeout(0.25)
            .andThen(intakeRollers.applyGoalCommand(IntakeRollersGoal.STOP)));
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
                        () -> flywheel.setVoltage(Dashboard.manualFlywheelVelocity.get()),
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
  }
}
