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
        public static final double CruiseVelocity = 80;
        public static final double ACceleration = 160;
        public static final double Jerk = 1600;


        public static final double kS = 0.25;
        public static final double kV = 0.2;
        public static final double kA = 0.01;
        public static final double kp = 4.8;
        public static final double ki = 0.05;
        public static final double kd = 0.1;
        /** The CAN ID for the motor */
        public static final int motorCanId = 0;
        public static final double tolerance = 0.025;

        public static final double m_steps = 1024.0;

        public static final int channel_a = 0;
        public static final int channel_b = 1;

        public static final int channel_a_a = 2;
        public static final int channel_b_b = 3;

        public static final int gear_ratio_on_drive_ring = 90;
    }

    /** Constants for intake */
    public static class IntakeConstants {

    }

    /** Constants for storage rollers */
    public static class RollerConstants {

    }

    /** Constants for climber */
    public static class ClimbConstants {

    }

    /** Constants for LEDs */
    public static class LedConstants {

    }
}
