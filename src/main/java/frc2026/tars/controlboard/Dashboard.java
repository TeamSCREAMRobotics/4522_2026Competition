package frc2026.tars.controlboard;

import com.teamscreamrobotics.dashboard.DashboardBoolean;
import com.teamscreamrobotics.dashboard.DashboardNumber;

public class Dashboard {

  private static final String overrides = "Overrides";

  public static DashboardBoolean ferryMode;
  public static DashboardBoolean aimAtHub;
  public static DashboardBoolean onlyUsePoseForHub;

  public static DashboardBoolean zeroIntake;
  public static DashboardBoolean zeroClimber;
  public static DashboardBoolean zeroHood;

  public static DashboardBoolean unClogFeeder;
  public static DashboardBoolean disableWaitUntilAtVelocity;

  public static DashboardBoolean manualMode;
  public static DashboardBoolean resetManuals;

  public static DashboardNumber manualTurretAngle;
  public static DashboardNumber manualHoodAngle;
  public static DashboardNumber manualFlywheelVelocity;
  public static DashboardNumber manualCLimber;
  public static DashboardNumber manualIntakeRollers;
  public static DashboardNumber manualIntakeWrist;
  public static DashboardNumber manualIndexer;

  public static DashboardBoolean autoShoot;
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

  static {
    initialize();
  }

  public static void initialize() {
    disableAmbiguityRejection = new DashboardBoolean(vision, "Disable Ambiguity Rejection", false);
    disableAllVisionUpdates = new DashboardBoolean(vision, "Disable All Vision Updates", false);
    useGlobalEstimateForAutoAlign =
        new DashboardBoolean(vision, "Use Global Estimate For Auto Align", false);
    disableVisionRequirement = new DashboardBoolean(vision, "Disable Coral Requirement", false);

    ferryMode = new DashboardBoolean(overrides, "Manual Ferry Mode", false);
    zeroIntake = new DashboardBoolean(overrides, "Zero Intake", false);
    zeroClimber = new DashboardBoolean(overrides, "Zero Climber", false);
    zeroHood = new DashboardBoolean(overrides, "Zero Hood", false);
    unClogFeeder = new DashboardBoolean(overrides, "Reset Field Centric", false);
    disableWaitUntilAtVelocity =
        new DashboardBoolean(overrides, "Disable Wait Until At Velocity", false);
    manualMode = new DashboardBoolean(overrides, "Manual Mode", false);
    resetManuals = new DashboardBoolean(overrides, "Reset Manuals", false);
    manualTurretAngle = new DashboardNumber(overrides, "Manual Turret Angle", 0.0);
    manualHoodAngle = new DashboardNumber(overrides, "Manual Hood Angle", 0.0);
    manualFlywheelVelocity = new DashboardNumber(overrides, "Manual Flywheel Velocity", 0.0);
    manualCLimber = new DashboardNumber(overrides, "Manual Climber", 0.0);
    manualIntakeRollers = new DashboardNumber(overrides, "Manual Intake Rollers", 0.0);
    manualIntakeWrist = new DashboardNumber(overrides, "Manual Intake Wrist", 0.0);
    manualIndexer = new DashboardNumber(overrides, "Manual Indexer", 0.0);
    autoShoot = new DashboardBoolean(overrides, "Auto Shoot", false);
    bumperShoot = new DashboardBoolean(overrides, "Bumper Shoot", false);

    tuningMode = new DashboardBoolean(tuning, "Tuning Mode", false);
    hoodAngle = new DashboardNumber(tuning, "Hood Angle", 0.0);
    flywheelVelocity = new DashboardNumber(tuning, "Flywheel Velocity", 0.0);
  }

  public static void resetManuals() {
    manualTurretAngle.set(0.0);
    manualHoodAngle.set(0.0);
    manualFlywheelVelocity.set(0.0);
    manualCLimber.set(0.0);
    manualIntakeRollers.set(0.0);
    manualIntakeWrist.set(0.0);
    manualIndexer.set(0.0);
  }
}
