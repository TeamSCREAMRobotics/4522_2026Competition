package frc2026.robot;

import com.teamscreamrobotics.gameutil.GameState;
import frc2026.robot.controlboard.Controlboard;
import java.util.function.DoubleSupplier;

public class RobotState {
  public enum Mode {
    AUTO,
    TELEOP,
    SHOOTINGTOHUB,
    FERRYING,
    CLIMBING,
    NOTHING
  }

  public Mode getMode() {
    Mode currentMode = Mode.NOTHING;

    if (GameState.determineGameState().toString() == "AUTO") {
      currentMode = Mode.AUTO;
    } else {

    }

    return currentMode;
  }

  // TODO: ACTUALLY DO

  public static DoubleSupplier getSpeedLimit() {
    return () -> {
      if (false) {
        return 1;
      } else if (Controlboard.driveController.getLeftTriggerAxis()
          > Controlboard.TRIGGER_DEADBAND) {
        return 0.5;
      } else {
        return 1;
      }
    };
  }
}
