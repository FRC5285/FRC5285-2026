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
        public static final int intakeID = 21;
        public static final int lowerID = 22;
        public static final int encoderChannel = 2;
        
        /** Encoder value when it is supposed to be at 0 (0 is when the intake is flat) */
        public static final double encoderStartValue = 0.0;

        public static final double intakeLoweredValue = 0.0;

        public static final double intakeRaisedValue = 0.75 * 2.0 * Math.PI;

        public static final double gearRatio = 1.0 / 5.0; // motor -> encoder

        public static final double kS = 0.0;
        public static final double kG = 0.0;
        public static final double kV = 0.0;
        public static final double kP = 8.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double maxVel = 1.0; // for trapezoidal profile; max velocity in rotations/second
        public static final double maxAcc = 2.0; // max acceleration in rotations/second/second

        // intake settings
        public static final double intakeVolts = 8.0;

    }

    /** Constants for turret intake */
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
