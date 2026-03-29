package frc2026.tars.controlboard;

import com.teamscreamrobotics.dashboard.DashboardBoolean;
import com.teamscreamrobotics.dashboard.DashboardNumber;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc2026.tars.subsystems.shooter.ShooterConstants;

public class Dashboard {

  private static final String overrides = "Overrides";

  public static DashboardBoolean aimAtHub;
  public static DashboardBoolean disableWaitUntilAim;
  public static DashboardBoolean disableWaitUntilHood;
  public static DashboardBoolean runBackIntake;
  public static DashboardBoolean runBackFlywheel;

  public static DashboardBoolean zeroIntake;
  public static DashboardBoolean zeroHood;
  public static DashboardBoolean zeroHopper;

  public static DashboardBoolean disableWaitUntilAtVelocity;

  public static DashboardBoolean manualMode;
  public static DashboardBoolean resetManuals;

  public static DashboardNumber manualHoodAngle;
  public static DashboardNumber manualFlywheelVelocity;
  public static DashboardNumber manualIntakeRollers;
  public static DashboardNumber manualIntakeWrist;
  public static DashboardNumber manualFloorRollers;
  public static DashboardNumber manualFeeder;

  public static DashboardBoolean bumperShoot;

  private static final String vision = "Vision";

  public static DashboardBoolean disableVisionRequirement;
  public static DashboardBoolean disableAmbiguityRejection;
  public static DashboardBoolean disableAllVisionUpdates;
  public static DashboardBoolean useGlobalEstimateForAutoAlign;

  private static final String tuning = "Tuning";

  public static DashboardBoolean tuningMode;
  public static DashboardNumber hoodAngle;
  public static DashboardNumber flywheelVelocity;

  public static DashboardNumber closeMapNudge;
  public static DashboardNumber midMapNudge;
  public static DashboardNumber farMapNudge;

  public static DashboardBoolean hailMaryMode;

  public static DashboardNumber functionCurve;
  public static DashboardNumber functionScalar;

  private static Field2d field = new Field2d();

  static {
    initialize();
  }

  public static void initialize() {
    zeroHopper = new DashboardBoolean(overrides, "Zero Hopper", false);
    disableAmbiguityRejection = new DashboardBoolean(vision, "Disable Ambiguity Rejection", false);
    disableAllVisionUpdates = new DashboardBoolean(vision, "Disable All Vision Updates", false);
    runBackIntake = new DashboardBoolean(overrides, "Run Back Intake", false);
    runBackFlywheel = new DashboardBoolean(overrides, "Run Back Flywheel", false);
    disableWaitUntilHood = new DashboardBoolean(overrides, "Disable Wait Until Hood", false);

    zeroIntake = new DashboardBoolean(overrides, "Zero Intake", false);
    // zeroClimber = new DashboardBoolean(overrides, "Zero Climber", false);
    zeroHood = new DashboardBoolean(overrides, "Zero Hood", false);
    disableWaitUntilAtVelocity =
        new DashboardBoolean(overrides, "Disable Wait Until At Velocity", false);
    manualMode = new DashboardBoolean(overrides, "Manual Mode", false);
    resetManuals = new DashboardBoolean(overrides, "Reset Manuals", false);
    manualHoodAngle = new DashboardNumber(overrides, "Manual Hood Angle", 0.0);
    manualFlywheelVelocity = new DashboardNumber(overrides, "Manual Flywheel Velocity", 0.0);
    manualIntakeRollers = new DashboardNumber(overrides, "Manual Intake Rollers", 0.0);
    manualIntakeWrist = new DashboardNumber(overrides, "Manual Intake Wrist", 90.0);
    manualFloorRollers = new DashboardNumber(overrides, "Manual Floor Rollers", 0);
    manualFeeder = new DashboardNumber(overrides, "Manual Feeder", 0);
    // autoShoot = new DashboardBoolean(overrides, "Auto Shoot", false);
    bumperShoot = new DashboardBoolean(overrides, "Bumper Shoot", false);
    disableWaitUntilAim = new DashboardBoolean(overrides, "Dissable Wait until aim", false);

    closeMapNudge =
        new DashboardNumber(overrides, "Close Tree Map Nudge", ShooterConstants.CLOSE_MAP_NUDGE);
    midMapNudge =
        new DashboardNumber(overrides, "Middle Tree Map Nudge", ShooterConstants.MID_MAP_NUDGE);
    farMapNudge =
        new DashboardNumber(overrides, "Far Tree Map Nudge", ShooterConstants.FAR_MAP_NUDGE);

    hailMaryMode = new DashboardBoolean(overrides, "Hail Mary Mode", false);

    tuningMode = new DashboardBoolean(tuning, "Tuning Mode", false);
    hoodAngle = new DashboardNumber(tuning, "Hood Angle", 0.0);
    flywheelVelocity = new DashboardNumber(tuning, "Flywheel Velocity", 0.0);
    functionCurve = new DashboardNumber(tuning, "Function Curve", ShooterConstants.FUNCTION_CURVE);
    functionScalar =
        new DashboardNumber(tuning, "Function Scalar", ShooterConstants.FUNCTION_SCALAR);
  }

  public static void resetManuals() {
    manualHoodAngle.set(0.0);
    manualFlywheelVelocity.set(0.0);
    manualIntakeRollers.set(0.0);
    manualIntakeWrist.set(90.0);
    manualFloorRollers.set(0.0);
    manualFeeder.set(0.0);
  }

  public static void periodic() {
    SmartDashboard.putData(field);
  }
}
