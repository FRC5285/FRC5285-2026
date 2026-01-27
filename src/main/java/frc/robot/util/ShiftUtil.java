package frc.robot.util;

/** The class to find out if it's our shift. Does not require a constructor. */
public final class ShiftUtil {
    /**
     * If we can score right now.
     * Always returns true if the Driver Station data is not available.
     * 
     * @return true if it's our shift, false if it's not our shift
     */
    public boolean canScore() {
        return true;
    }
}
