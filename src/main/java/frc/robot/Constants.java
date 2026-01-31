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
        
        /*
        ***CLIMB CONSTANTS FROM 2025 ROBOT***
        ---------------------------------------
        public static final int motorID = 17; // CAN
        public static final double climbSpeed = 0.70;
        public static final double climbRotations = 63.1; // 77



        ***ELEVATOR CONSTANTS FROM 2025 ROBOT***
        --------------------------------------------
        public static final int elevatorMotorID = 13; // CAN
        public static final int followMotorID = 14; // CAN
        // public static final int topLimitSwitchID = 1; // DIO
        // public static final int bottomLimitSwitchID = 2; // DIO

        public static final int encoderA = 1; // DIO, Blue
        public static final int encoderB = 2; // DIO, Yellow

        public static final double kP = 6.0; //4.0
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kS = 0.0;

        ***If this one is too low, elevator goes slamming down. If too high, elevator goes slamming up. Get this one right.

        public static final double kG = 0.325; // 0.4225 // Get this one to 3 decimal places of precision, 0.129 is STARTING POINT ONLY!!!
        public static final double kV = 3.4; // 3.6
        public static final double kA = 0.0;

        public static final double maxV = 3.5; // Max velocity, 2.5
        public static final double maxA = 2.5; // Max acceleration

        public static final double elevatorGearRadius = 0.022;
        public static final double encoderPulseDist = (2.0 * Math.PI * elevatorGearRadius) / 1024;

        public static final double level1Position = 0.15;
        public static final double level3Position = 0.75; // 0.7827
        public static final double level2Position = level3Position - 0.4; // 0.3625
        public static final double level4Position = 1.335;
        public static final double intakePosition = 0.12; // 0.21
        public static final double floorAlgaePosition = 0.0;
        
        public static final double L2AlgaeHeight = 0.0; // 0.608
        public static final double L3AlgaeHeight = 0.375; // 0.956
        public static final double elevatorAlgaeMoveUpAmount = 0.15;

        public static final double maxHeight = 1.335;
        public static final double minHeight = 0.0;

        public static final double processorHeight = 0.2;

        // Elevator is at goal position if it is this close to the goal position
        public static final double goalRange = 0.025;

        public static final double encoderOffset = 0.049;
        */
    }

    /** Constants for LEDs */
    public static class LedConstants {

    }
}
