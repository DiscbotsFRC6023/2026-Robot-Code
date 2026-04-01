package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;

public class GameState {

    /** Seconds before a state transition to start rumbling the controller. */
    private static final double RUMBLE_WARNING_TIME_SECONDS = 3.0;

    String gameData = DriverStation.getGameSpecificMessage();

    public enum States{
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

    public static States getGameState(){
        // Check autonomous first (2:20 to 2:00)
        if (DriverStation.isAutonomous()) {
        return States.AUTO;
        }

        // Check if we're not enabled
        if (!DriverStation.isTeleop()) {
        return States.STANDING;
        }

        // Teleop time checks (match time counts DOWN from 2:30)
        double timeRemaining = DriverStation.getMatchTime();

        if (timeRemaining > 120 && timeRemaining <= 130) {
        return States.TRANSITION;
        } else if (timeRemaining > 105 && timeRemaining <= 130) {
        return States.SHIFTONE;
        } else if (timeRemaining > 80 && timeRemaining <= 105) {
        return States.SHIFTTWO;
        } else if (timeRemaining > 55 && timeRemaining <= 80) {
        return States.SHIFTTHREE;
        } else if (timeRemaining > 30 && timeRemaining <= 55) {
        return States.SHIFTFOUR;
        } else if (timeRemaining > 0 && timeRemaining <= 30) {
        return States.ENDGAME;
        }

        return States.STANDING;
    }

    public States getActiveHub(){
        if(gameData.length() > 0){      // Game data has been received
            if(gameData.charAt(0) == 'B'){
                return States.BLUE;
            } else if(gameData.charAt(0) == 'R'){
                return States.RED;
            } else {
                return null;
            }
        }
        return null;
    }

    public static double timeRemainingInCurrentState(){
        States currState = getGameState();
        double time = DriverStation.getMatchTime();
        switch (currState) {
      case AUTO:
        // AUTO: 2:20 to 2:00 (150 to 120)
        return time;
      case TRANSITION:
        // TRANSITION: 2:00 to 2:10 (120 to 130)
        return Math.max(0, time - 120);
      case SHIFTONE:
        // SHIFTONE: 2:10 to 1:45 (130 to 105)
        return Math.max(0, time - 105);
      case SHIFTTWO:
        // SHIFTTWO: 1:45 to 1:20 (105 to 80)
        return Math.max(0, time - 80);
      case SHIFTTHREE:
        // SHIFTTHREE: 1:20 to 0:55 (80 to 55)
        return Math.max(0, time - 55);
      case SHIFTFOUR:
        // SHIFTFOUR: 0:55 to 0:30 (55 to 30)
        return Math.max(0, time - 30);
      case ENDGAME:
        // ENDGAME: 0:30 to 0:00 (30 to 0)
        return Math.max(0, time);
      default:
        return 0;
    }
    }

    /**
     * Returns true when the match is within {@link #RUMBLE_WARNING_TIME} seconds
     * of transitioning to the next teleop game state. Should be called periodically
     * during teleop to drive controller rumble.
     */
    public static boolean shouldRumble() {
        States state = getGameState();
        // Only rumble during teleop states (not AUTO or STANDING)
        if (state == States.AUTO || state == States.STANDING) {
            return false;
        }
        double remaining = timeRemainingInCurrentState();
        return remaining <= RUMBLE_WARNING_TIME_SECONDS && remaining > 0;
    }
} 
