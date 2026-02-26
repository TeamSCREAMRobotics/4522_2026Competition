package frc2026.tars.util;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class Util {
  public static Transform2d transform3dTo2dXY(Transform3d transform) {
    return new Transform2d(
        transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
  }
}
