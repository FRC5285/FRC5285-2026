package frc.robot;

public final class Constants {
    /** Constants relating to the driver input */
    public static class OperatorConstants {
        /** The driver station port to use as the driver controller port */
        public static final int driverControllerPort = 0;

        /** The deadband for the driver controller joysticks */
        public static final double driveDeadband = 0.1;
    }

    /** Constants for use during Auto */
    public static class AutoConstants {

    }

    /** Constants for vision */
    public static class VisionConstants {

    }

    /** Constants for turret */
    public static class TurretConstants {

    }

    /** Constants for intake */
    public static class IntakeConstants {

    }

    /** Constants for storage rollers */
    public static class RollerConstants {

    }

    /** Constants for climber */
    public static class ClimbConstants {
        
        //MOTOR CONSTANTS
        public static final int motorID = 17; // CAN

        public static final double kP = 6.0; //4.0
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double maxV = 3.5; // Max velocity, 2.5
        public static final double maxA = 2.5; // Max acceleration

        //LIDAR CONSTANTS
        public static final double maxExtension = 0.0; //distance between sensor and plate (meters)

        public static final double minExtension = 0.0; //distance between sensor and plate (meters)

        public static final double lidarOffset = 0.010; // Distance of lidar measurement - real measurement // 0.025

    }

    /** Constants for LEDs */
    public static class LedConstants {

    }
}
