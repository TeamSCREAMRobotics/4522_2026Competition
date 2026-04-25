package frc2026.tars.subsystems.shooter;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  public static final double HEIGHT = 0.510697; // meters

  public static final InterpolatingDoubleTreeMap OLD_FLYWHEEL_MAP =
      new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap NEW_FLYWHEEL_MAP =
      new InterpolatingDoubleTreeMap();

  public static final InterpolatingDoubleTreeMap NEW_NEW_FLYWHEEL_MAP =
      new InterpolatingDoubleTreeMap();

  public static final double LATENCY = 0.15;

  // Shoot-on-the-fly: control loop phase delay (s) and aerodynamic drag constant (1/s)
  public static final double SOTF_PHASE_DELAY = 0.03;
  public static final double SOTF_LINEAR_DRAG_CONSTANT = 0.375;

  public static final double FUNCTION_CURVE = 0.01;
  public static final double FUNCTION_SCALAR = 4.1;

  public static final double CLOSE_MAP_NUDGE = 1;
  public static final double MID_MAP_NUDGE = 1;
  public static final double FAR_MAP_NUDGE = 1;

  public static final Transform3d flywheelToRobot =
      new Transform3d(
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(-10.098),
          Units.inchesToMeters(16.921),
          Rotation3d.kZero);

  static {
    // Put in distance in meters, gets out Flywheel RPS
    OLD_FLYWHEEL_MAP.put(1.472, 40.5);
    OLD_FLYWHEEL_MAP.put(1.806, 44.0);
    OLD_FLYWHEEL_MAP.put(1.963, 46.0);
    OLD_FLYWHEEL_MAP.put(2.145, 47.0);
    OLD_FLYWHEEL_MAP.put(2.370, 48.5);
    OLD_FLYWHEEL_MAP.put(2.565, 49.5);
    OLD_FLYWHEEL_MAP.put(2.840, 51.0);
    OLD_FLYWHEEL_MAP.put(3.183, 52.0);
    OLD_FLYWHEEL_MAP.put(3.306, 52.75);
    OLD_FLYWHEEL_MAP.put(3.528, 53.5);
    OLD_FLYWHEEL_MAP.put(3.799, 54.5);
    OLD_FLYWHEEL_MAP.put(4.003, 55.2);
    OLD_FLYWHEEL_MAP.put(4.224, 55.8);
    OLD_FLYWHEEL_MAP.put(4.472, 56.0);
    OLD_FLYWHEEL_MAP.put(4.739, 56.2);
    OLD_FLYWHEEL_MAP.put(5.024, 56.75);
    OLD_FLYWHEEL_MAP.put(5.277, 57.0);
    OLD_FLYWHEEL_MAP.put(5.426, 57.1);
    OLD_FLYWHEEL_MAP.put(5.617, 57.2);

    NEW_FLYWHEEL_MAP.put(1.770, 29.93);
    NEW_FLYWHEEL_MAP.put(1.985, 31.53);
    NEW_FLYWHEEL_MAP.put(2.196, 32.06);
    NEW_FLYWHEEL_MAP.put(2.391, 32.0);
    NEW_FLYWHEEL_MAP.put(2.419, 32.5);
    NEW_FLYWHEEL_MAP.put(2.685, 33.0);
    NEW_FLYWHEEL_MAP.put(2.948, 33.5);
    NEW_FLYWHEEL_MAP.put(3.158, 34.0);
    NEW_FLYWHEEL_MAP.put(3.291, 34.5);
    NEW_FLYWHEEL_MAP.put(3.546, 35.0);
    NEW_FLYWHEEL_MAP.put(3.783, 35.75);
    NEW_FLYWHEEL_MAP.put(4.002, 36.3);
    NEW_FLYWHEEL_MAP.put(4.212, 36.89);
    NEW_FLYWHEEL_MAP.put(4.401, 37.4);
    NEW_FLYWHEEL_MAP.put(4.610, 37.9);
    NEW_FLYWHEEL_MAP.put(4.829, 38.6);
    NEW_FLYWHEEL_MAP.put(5.068, 39.3);
    NEW_FLYWHEEL_MAP.put(5.206, 40.2);
    // Ferrying Dist. Keep closer so fuel doesn't scatter.
    NEW_FLYWHEEL_MAP.put(5.477, 41.0);
    NEW_FLYWHEEL_MAP.put(5.727, 42.0);
    NEW_FLYWHEEL_MAP.put(5.944, 43.5);
    NEW_FLYWHEEL_MAP.put(6.265, 45.0);
    // Running anything harder on the flywheel is to much for the battery.

    NEW_NEW_FLYWHEEL_MAP.put(1.770, 29.93);
    NEW_NEW_FLYWHEEL_MAP.put(1.985, 31.0);
    NEW_NEW_FLYWHEEL_MAP.put(2.196, 31.25);
    NEW_NEW_FLYWHEEL_MAP.put(2.414, 31.6);
    NEW_NEW_FLYWHEEL_MAP.put(2.6204, 32.1);
    NEW_NEW_FLYWHEEL_MAP.put(2.8214, 32.6);
    NEW_NEW_FLYWHEEL_MAP.put(3.0268, 33.3);
    NEW_NEW_FLYWHEEL_MAP.put(3.26803, 34.0);
    NEW_NEW_FLYWHEEL_MAP.put(3.4405, 35.2);
    NEW_NEW_FLYWHEEL_MAP.put(3.6298, 35.9);
    NEW_NEW_FLYWHEEL_MAP.put(3.8246, 36.8);
    NEW_NEW_FLYWHEEL_MAP.put(3.98114, 37.4);
    NEW_NEW_FLYWHEEL_MAP.put(4.2055, 39.0);
    NEW_NEW_FLYWHEEL_MAP.put(4.4453, 39.9);
    NEW_NEW_FLYWHEEL_MAP.put(4.6694, 41.0);
    NEW_NEW_FLYWHEEL_MAP.put(4.80105, 42.0);
    NEW_NEW_FLYWHEEL_MAP.put(5.1055, 43.2);

    NEW_FLYWHEEL_MAP.put(5.477, 41.0);
    NEW_FLYWHEEL_MAP.put(5.727, 42.0);
    NEW_FLYWHEEL_MAP.put(5.944, 43.5);
    NEW_FLYWHEEL_MAP.put(6.265, 45.0);

    // NEW_NEW_FLYWHEEL_MAP.put

  }

  public static final CANrangeConfiguration beamConfig = new CANrangeConfiguration();

  static {
    beamConfig.ProximityParams.ProximityThreshold = 0.61;
  }
}
