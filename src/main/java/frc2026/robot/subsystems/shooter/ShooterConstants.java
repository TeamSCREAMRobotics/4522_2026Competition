package frc2026.robot.subsystems.shooter;

import com.teamscreamrobotics.data.Length;

public class ShooterConstants {
  public static final double HEIGHT =
      Length.fromInches(35.0).getMeters(); // ~35 inches (example mounting height)

  public static final double GRAVITY = 9.81;
  public static final double FUEL_MASS = 0.215;
  public static final double FUEL_DIAMETER = 0.15;
  public static final double AIR_DENSITY = 1.225;
  public static final double DRAG_COEFFICIENT = 0.47;
}
