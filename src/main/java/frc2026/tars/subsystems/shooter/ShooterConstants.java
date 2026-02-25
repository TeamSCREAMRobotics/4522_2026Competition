package frc2026.tars.subsystems.shooter;

import com.teamscreamrobotics.gameutil.ShootStateInterpolatingTreeMap;
import com.teamscreamrobotics.gameutil.ShootStateInterpolatingTreeMap.ShootState;

public class ShooterConstants {
  public static final double HEIGHT = 0.510697; // meters

    public static final ShootStateInterpolatingTreeMap SHOOTER_DIST_MAP = new ShootStateInterpolatingTreeMap();

    static{
      // TODO: Tune and find values.
      SHOOTER_DIST_MAP.put(0.0, new ShootState(0.0,0.0));
      SHOOTER_DIST_MAP.put(0.5, new ShootState(0.5,0.5));
    }

    
}
