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
    
        //**************CLIMB MOTOR CONSTANTS************
        public static final int climbMotorID = 17; //need to adjust

        public static final double ckP = 6.0; //need to calibrate
        public static final double ckI = 0.0; //need to calibrate
        public static final double ckD = 0.0; //need to calibrate

        public static final double cmaxV = 3.5; //need to calibrate
        public static final double cmaxA = 2.5; //need to calibrate


        //***************ROTATE MOTOR CONSTANTS***************
        public static final int rotateMotorID = 16; //need to adjust

        public static final double rotateGoalRotations = 0.25; //need to calibrate

        public static final double rkP = 6.0; //need to calibrate
        public static final double rkI = 0.0; //need to calibrate
        public static final double rkD = 0.0; //need to calibrate

        public static final double rmaxV = 3.5; //need to calibrate
        public static final double rmaxA = 2.5; //need to calibrate

        
        //**************LIDAR CONSTANTS************************
        public static final double maxExtension = 0.307975; //12.125 inches -- //distance between sensor and plate (meters)

        public static final double minExtension = 0.1143; //4.5 inches -- //distance between sensor and plate (meters)

        public static final double lidarOffset = 0.010; //need to calibrate -- Distance of lidar measurement - real measurement // 0.025

    }

    /** Constants for LEDs */
    public static class LedConstants {

    }
}
