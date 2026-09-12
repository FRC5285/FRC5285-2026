package frc.robot.util;

import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import frc.robot.Constants.LEDConstants;

/** The class to find out if it's our shift. Does not require a constructor. */
public final class ShiftUtil {
    private static String currentGameData = "";

    /**
     * If we can score right now.
     * Always returns true if the Driver Station data is not available.
     * 
     * @return true if it's our shift, false if it's not our shift
     */
    public static boolean canScore() {
        Alliance activeAlliance = getActiveAlliance();
        Alliance teamAlliance = MatchState.getAlliance().orElse(Alliance.BLUE);

        if (activeAlliance == null) {
            return true;
        }

        return activeAlliance == teamAlliance;
    }

    /**
     * If it's a number of seconds before shooting
     * 
     * @return yes or no
     */
    public static boolean beforeShooting() {
        Alliance aboutShootAlliance = getAboutToShootAlliance();
        Alliance teamAlliance = MatchState.getAlliance().orElse(Alliance.BLUE);

        if (aboutShootAlliance == null) {
            return false;
        }

        return aboutShootAlliance == teamAlliance;
    }

    public static Alliance getActiveAlliance() {
        if (RobotState.isAutonomous()) {
            return MatchState.getAlliance().orElse(Alliance.BLUE);
        }

        var currentTime = MatchState.getMatchTime();

        if (currentGameData.length() == 0) {
            currentGameData = MatchState.getGameData().get();
            if (currentGameData.length() == 0) {
                return null;
            }
        }

        Alliance initialAlliance = MatchState.getGameData().get().charAt(0) == 'R' ? Alliance.RED : Alliance.BLUE;

        if (currentTime >= 130 || currentTime < 30) {
            return MatchState.getAlliance().orElse(Alliance.BLUE);
        }
        else if (currentTime >= 105 || (currentTime < 80 && currentTime >= 55)) {
            return initialAlliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
        }
        else {
            return initialAlliance;
        }
    }

    public static Alliance getAboutToShootAlliance() {
        if (RobotState.isAutonomous()) {
            return MatchState.getAlliance().orElse(Alliance.BLUE);
        }

        var currentTime = MatchState.getMatchTime();

        if (currentGameData.length() == 0) {
            currentGameData = MatchState.getGameData().get();
            if (currentGameData.length() == 0) {
                return null;
            }
        }

        Alliance initialAlliance = MatchState.getGameData().get().charAt(0) == 'R' ? Alliance.RED : Alliance.BLUE;

        if (withinSomeNumberLess(130, currentTime)) {
            return MatchState.getAlliance().orElse(Alliance.BLUE);
        }
        else if (withinSomeNumberLess(105, currentTime) || withinSomeNumberLess(55, currentTime)) {
            return initialAlliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
        }
        else {
            return initialAlliance;
        }
    }

    private static boolean withinSomeNumberLess(double compareToNumber, double variableNumber) {
        return variableNumber <= compareToNumber && compareToNumber - variableNumber <= LEDConstants.shiftAlmostNum;
    }

    public static void resetShift() {
        currentGameData = "";
    }
}
