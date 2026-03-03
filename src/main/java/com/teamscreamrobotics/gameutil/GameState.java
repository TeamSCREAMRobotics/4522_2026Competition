package com.teamscreamrobotics.gameutil;

import edu.wpi.first.wpilibj.DriverStation;

public class GameState {
  public enum States {
    AUTO,
    TRANSITION,
    SHIFTONEFLASHING,
    SHIFTONE,
    SHIFTTWOFLASHING,
    SHIFTTWO,
    SHIFTTHREEFLASHING,
    SHIFTTHREE,
    SHIFTFOURFLASHING,
    SHIFTFOUR,
    ENDGAMEFLASHING,
    ENDGAME,
    BLUE,
    RED,
    STANDING
  }

  public States getActiveHub() {
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData != null && gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          return States.BLUE;
        case 'R':
          return States.RED;
      }
    }

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red ? States.RED : States.BLUE;
    }

    return States.BLUE;
  }

  public static States determineGameState() {
    if (DriverStation.isAutonomous()) {
      return States.AUTO;
    }

    if (!DriverStation.isTeleopEnabled()) {
      return States.STANDING;
    }

    double timeRemaining = DriverStation.getMatchTime();

    if (timeRemaining > 147) {
      return States.TRANSITION;
    } else if (timeRemaining > 144) {
      return States.SHIFTONEFLASHING;
    } else if (timeRemaining > 119) {
      return States.SHIFTONE;
    } else if (timeRemaining > 116) {
      return States.SHIFTTWOFLASHING;
    } else if (timeRemaining > 91) {
      return States.SHIFTTWO;
    } else if (timeRemaining > 88) {
      return States.SHIFTTHREEFLASHING;
    } else if (timeRemaining > 63) {
      return States.SHIFTTHREE;
    } else if (timeRemaining > 60) {
      return States.SHIFTFOURFLASHING;
    } else if (timeRemaining > 35) {
      return States.SHIFTFOUR;
    } else if (timeRemaining > 32) {
      return States.ENDGAMEFLASHING;
    } else if (timeRemaining >= 0) {
      return States.ENDGAME;
    }

    return States.STANDING;
  }
}
