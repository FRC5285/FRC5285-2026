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
        public static final double kS = 0.0; //turret base
        public static final double kV = 11.2;
        public static final double kp = 80.0;
        public static final double ki = 0.0;
        public static final double kd = 0.0;
        public static final double turretMaxV = 3.0;
        public static final double turretMaxA = 5.0;
        
        public static final double S_ACceleration = 320;
        public static final double S_Jerk = 420.69;

        public static final double S_kS = 0.17255; //shooter
        public static final double S_kV = 0.11311;
        public static final double S_kA = 0.0050611;
        public static final double S_kp = 0.10589;
        public static final double S_ki = 0.0;
        public static final double S_kd = 0.0;
        /** The CAN ID for the motor */
        public static final int motorCanId = 15;
        public static final int ShooterMotorCanId = 16;
        public static final int ShooterMotor2CanId = 17;

        public static final int channel_a = 0;
        public static final int channel_b = 1;
 
        public static final double encoderAOffset = -0.746;
        public static final double encoderBOffset = -0.0439;
        public static final double max_range = 0.5; //rotations
        public static final double min_range = -0.5; //rotaitons
        public static final double match_tolerance = 0.02; //rotations

        public static final double turretPIDMin = -0.25;
        public static final double turretPIDMax = 0.25;
    }

    /** Constants for intake */
    public static class IntakeConstants {

    }

    /** Constants for turret intake */
    public static class TurretIntakeConstants {
        public static final int motorCanId = 18;

        // arbitrary values (for now)
        public static final double kS = 0.0;
        public static final double kV = 0.1;
        public static final double kA = 0.0;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        // motion magic settings
        public static final double cruiseVelocity = 80;
        public static final double acceleration = 320;
        public static final double jerk = 1600;

        public static final double intakeSpeed = 120.0;
        public static final double reverseSpeed = -40.0;

        public static final double speedTolerance = 16.7;
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
