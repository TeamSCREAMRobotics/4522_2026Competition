package frc2026.tars.subsystems.shooter;

import com.ctre.phoenix6.configs.CANrangeConfiguration;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  public static final double HEIGHT = 0.510697; // meters

  public static final InterpolatingDoubleTreeMap OLD_FLYWHEEL_MAP = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap NEW_FLYWHEEL_MAP = new InterpolatingDoubleTreeMap();

  public static final double LATENCY = 0.15;

  public static final double FUNCTION_CURVE = 0.01;
  public static final double FUNCTION_SCALAR = 4.1;

  public static final double CLOSE_MAP_NUDGE = 1;
  public static final double MID_MAP_NUDGE = 1;
  public static final double FAR_MAP_NUDGE = 1;

  public static final Transform3d flywheelToRobot = new Transform3d(Units.inchesToMeters(0.0),Units.inchesToMeters(-10.098),Units.inchesToMeters(16.921), Rotation3d.kZero);

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
    NEW_FLYWHEEL_MAP.put(2.419, 33.30);
    NEW_FLYWHEEL_MAP.put(2.685,33.0);
    NEW_FLYWHEEL_MAP.put(2.948, 33.5);
    NEW_FLYWHEEL_MAP.put(3.158,34.0);
    NEW_FLYWHEEL_MAP.put(3.291,34.5);
    NEW_FLYWHEEL_MAP.put(3.546, 35.0);
  }

  public static final CANrangeConfiguration beamConfig = new CANrangeConfiguration();

  static {
    beamConfig.ProximityParams.ProximityThreshold = 0.61;
  }
}
