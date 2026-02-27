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

    /** Constants for turret intake */
    public static class IntakeConstants {
        public static final int intakeID = 1; // change later\
        public static final int lowerID = 0;

        // arbitrary values (for now)

        // roller part
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 4.8;
        public static final double kI = 0;
        public static final double kD = 0.1;

        // lower part
        public static final double kS1 = 0.25;
        public static final double kV1 = 0.12;
        public static final double kA1 = 0.01;
        public static final double kP1 = 4.8;
        public static final double kI1 = 0;
        public static final double kD1 = 0.1;
        public static final double maxVel = 80; // for trapezoidal profile; max velocity
        public static final double maxAcc = 160; // max acceleration

        // motion magic settings
        public static final double cruiseVelocity = 80;
        public static final double acceleration = 160;
        public static final double jerk = 1600; // gooner

        // turret settings
        public static final double intakeSpeed = 160; // radians per second (i think?)

    }

    /** Constants for intake */
    public static class TurretIntakeConstants {
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
