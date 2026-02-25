package com.teamscreamrobotics.gameutil;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;
import lombok.Setter;

public class ShootStateInterpolatingTreeMap {
  private InterpolatingDoubleTreeMap angleInterpolator = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap velocityInterpolator = new InterpolatingDoubleTreeMap();

  public void put(double distance, ShootState shootState) {
    angleInterpolator.put(distance, shootState.getHoodDeg());
    velocityInterpolator.put(distance, shootState.getFlywheelRPS());
  }

  public ShootState get(double distance) {
    return new ShootState(angleInterpolator.get(distance), velocityInterpolator.get(distance));
  }

  public static class ShootState {

    @Getter @Setter double hoodDeg, flywheelRPS;

    public ShootState(double hoodDeg, double flywheelRPS) {
      this.hoodDeg = hoodDeg;
      this.flywheelRPS = flywheelRPS;
    }

    public ShootState() {
      this.hoodDeg = 0;
      this.flywheelRPS = 0;
    }
  }
}
