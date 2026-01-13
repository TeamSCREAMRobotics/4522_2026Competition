package frc2026.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class GameState {
  public enum States {
    AUTO,
    TRANSITION,
    SHIFTONE,
    SHIFTTWO,
    SHIFTTHREE,
    SHIFTFOUR,
    ENDGAME,
    BLUE,
    RED,
    STANDING
  }

  String gameData = DriverStation.getGameSpecificMessage();

  public States getActiveHub() {
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          return States.BLUE;
        case 'R':
          return States.RED;
        default:
          break;
      }
    } else {
      System.out.println("Code for no data received yet");
    }

    return null;
  }

  public static States determineGameState() {
        // Check autonomous first
        if (DriverStation.isAutonomous()) {
            return States.AUTO;
        }
        
        // Check if we're not enabled
        if (!DriverStation.isTeleop()) {
            return States.STANDING;
        }
        
        // Teleop time checks (match time counts DOWN from 150)
        double timeRemaining = DriverStation.getMatchTime();
        
        if (timeRemaining > 140) {
            return States.TRANSITION;
        } else if (timeRemaining > 130) {
            return States.SHIFTONE;
        } else if (timeRemaining > 105) {
            return States.SHIFTTWO;
        } else if (timeRemaining > 80) {
            return States.SHIFTTHREE;
        } else if (timeRemaining > 55) {
            return States.SHIFTFOUR;
        } else if (timeRemaining > 30) {
            return States.ENDGAME;
        }
        
        return States.STANDING;
    }

}
