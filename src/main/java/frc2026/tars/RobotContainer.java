package frc2026.tars;

import com.teamscreamrobotics.dashboard.MechanismVisualizer;
import com.teamscreamrobotics.util.Logger;
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
import frc2026.tars.subsystems.intake.IntakeWrist;
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
      Flywheel flywheel) {}

  private final IntakeWrist intakeWrist = new IntakeWrist(IntakeConstants.INTAKE_WRIST_CONFIG);
  private final IntakeRollers intakeRollers = new IntakeRollers(IntakeConstants.ROLLERS_CONFIG);
  private final Drivetrain drivetrain = TunerConstants.drivetrain;

  private final Shooter shooter = new Shooter(null, null);
  private final Turret turret = new Turret(TurretConstants.TURRET_CONFIG);
  private final Hood hood = new Hood(HoodConstants.HOOD_CONFIG);
  private final Flywheel flywheel = new Flywheel(FlywheelConstants.FLYWHEEL_CONFIG);
  private final Climber climber = new Climber(HoodConstants.HOOD_CONFIG);

  private final VisionManager visionManager = new VisionManager(drivetrain, turret);

  @Getter
  private final Subsystems subsystems =
      new Subsystems(drivetrain, intakeWrist, shooter, turret, hood, flywheel);

  @Getter private final RobotState robotState = new RobotState(subsystems);

  private final MechanismVisualizer mechVisualizer =
      new MechanismVisualizer(
          SimConstants.MEASURED_MECHANISM,
          SimConstants.SETPOINT_MECHANISM,
          RobotContainer::telemeterizeMechanisms,
          intakeWrist.intakeMech);

  public RobotContainer() {
    configureBindings();
    configureManualOverrides();
    configureDefaultCommands();

    SmartDashboard.putNumber("test", 1);

    mechVisualizer.setEnabled(true);
  }

  private void configureBindings() {
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

    intakeWrist.setDefaultCommand(
        intakeWrist.applyGoalCommand(IntakeWrist.IntakeWristGoal.STOW).withName("Stow"));

    intakeRollers.setDefaultCommand(
        intakeRollers.applyGoalCommand(IntakeRollers.IntakeRollersGoal.STOP).withName("Stopped"));
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
