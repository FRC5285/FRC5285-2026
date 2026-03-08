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

        public static final double S_ACceleration = 320;
        public static final double S_Jerk = 420.69;


        public static final double kS = 0.0; //turret base
        public static final double kV = 0.0;
        public static final double kp = 0.0; 
        public static final double ki = 0.0;
        public static final double kd = 0.0;
        public static final double turretMaxV = 3.0;
        public static final double turretMaxA = 5.0;

        public static final double S_kS = 0.25; //shooter
        public static final double S_kV = 0.8;
        public static final double S_kA = 0.01;
        public static final double S_kp = 0.1; 
        public static final double S_ki = 0.0;                                                                                                                                                                                                                              
        public static final double S_kd = 0.0;
        /** The CAN ID for the motor */
        public static final int motorCanId = 15;
        public static final int ShooterMotorCanId = 16;
        public static final int ShooterMotor2CanId = 17;
        public static final double tolerance = 0.025;

        public static final double m_steps = 1024.0;

        public static final int channel_a = 0;
        public static final int channel_b = 1;
 
        public static final double shooter_ratio = 1.0;

        public static final double convert_to_rotations_from_radians = 2 * Math.PI;

        public static final double encoderAOffset = -0.374;
        public static final double encoderBOffset = -0.601;
        public static final double max_range = 0.5; //rotations
        public static final double min_range = -0.5; //rotaitons
        public static final double match_tolerance = 0.02; //rotations
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
