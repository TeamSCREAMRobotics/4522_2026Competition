package frc2026.robot;

import com.teamscreamrobotics.gameutil.GameState;

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
}
