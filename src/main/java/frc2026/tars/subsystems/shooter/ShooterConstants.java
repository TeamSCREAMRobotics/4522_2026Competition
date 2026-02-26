package frc2026.tars.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
  public static final double HEIGHT = 0.510697; // meters

  public static final InterpolatingDoubleTreeMap FLYWHEEL_MAP = new InterpolatingDoubleTreeMap();

  public static final double FUNCTION_CURVE = 0.01;
  public static final double FUNCTION_SCALAR = 12.5;

  static{
    // Put in distance in meters, gets out Flywheel RPS
    FLYWHEEL_MAP.put(1.472, 40.5);
    FLYWHEEL_MAP.put(1.806, 44.0);
    FLYWHEEL_MAP.put(1.963, 46.0);
    FLYWHEEL_MAP.put(2.145, 47.0);
    FLYWHEEL_MAP.put(2.370, 48.5);
    FLYWHEEL_MAP.put(2.565, 49.5);
    FLYWHEEL_MAP.put(2.840, 51.0);
    FLYWHEEL_MAP.put(3.183, 52.0);
    FLYWHEEL_MAP.put(3.306, 52.75);
    FLYWHEEL_MAP.put(3.528, 53.5);
    FLYWHEEL_MAP.put(3.799, 54.5);
    FLYWHEEL_MAP.put(4.003, 55.2);
    FLYWHEEL_MAP.put(4.224, 55.8);
    FLYWHEEL_MAP.put(4.472, 56.0);
    FLYWHEEL_MAP.put(4.739, 56.2);
    FLYWHEEL_MAP.put(5.024, 56.75);
    FLYWHEEL_MAP.put(5.277, 57.0);
    FLYWHEEL_MAP.put(5.426, 57.1);
    FLYWHEEL_MAP.put(5.617, 57.2);

  }
}
