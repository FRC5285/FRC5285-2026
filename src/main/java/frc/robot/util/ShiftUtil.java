package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        Alliance teamAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

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
        Alliance teamAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (aboutShootAlliance == null) {
            return false;
        }

        return aboutShootAlliance == teamAlliance;
    }

    public static Alliance getActiveAlliance() {
        if (DriverStation.isAutonomous()) {
            return DriverStation.getAlliance().orElse(Alliance.Blue);
        }

        var currentTime = DriverStation.getMatchTime();

        if (currentGameData.length() == 0) {
            currentGameData = DriverStation.getGameSpecificMessage();
            if (currentGameData.length() == 0) {
                return null;
            }
        }

        Alliance initialAlliance = DriverStation.getGameSpecificMessage().charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;

        if (currentTime >= 130 || currentTime < 30) {
            return DriverStation.getAlliance().orElse(Alliance.Blue);
        }
        else if (currentTime >= 105 || (currentTime < 80 && currentTime >= 55)) {
            return initialAlliance == Alliance.Red ? Alliance.Blue : Alliance.Red;
        }
        else {
            return initialAlliance;
        }
    }

    public static Alliance getAboutToShootAlliance() {
        if (DriverStation.isAutonomous()) {
            return DriverStation.getAlliance().orElse(Alliance.Blue);
        }

        var currentTime = DriverStation.getMatchTime();

        if (currentGameData.length() == 0) {
            currentGameData = DriverStation.getGameSpecificMessage();
            if (currentGameData.length() == 0) {
                return null;
            }
        }

        Alliance initialAlliance = DriverStation.getGameSpecificMessage().charAt(0) == 'R' ? Alliance.Red : Alliance.Blue;

        if (withinSomeNumberLess(130, currentTime)) {
            return DriverStation.getAlliance().orElse(Alliance.Blue);
        }
        else if (withinSomeNumberLess(105, currentTime) || withinSomeNumberLess(55, currentTime)) {
            return initialAlliance == Alliance.Red ? Alliance.Blue : Alliance.Red;
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
