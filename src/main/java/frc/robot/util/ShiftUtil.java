package frc.robot.util;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** The class to find out if it's our shift. Does not require a constructor. */
public final class ShiftUtil {
    /**
     * If we can score right now.
     * Always returns true if the Driver Station data is not available.
     * 
     * @return true if it's our shift, false if it's not our shift
     */
    private static String currentGameData = "";

    public static boolean canScore() {
        Alliance activeAlliance = getActiveAlliance();
        Alliance teamAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

        if (activeAlliance == null) {
            return false;
        }

        return activeAlliance == teamAlliance;
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
}
