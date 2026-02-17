package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.generated.TunerConstants;

public final class Constants {
    /** Constants relating to the driver input */
    public static class OperatorConstants {
        /** The driver station port to use as the driver controller port */
        public static final int driverControllerPort = 0;

        /** The deadband for the driver controller joysticks */
        public static final double driveDeadband = 0.1;

        /** Minimum speed multiplier for throttle */
        public static final double throttleMinMultiplier = 0.1;

        /** Max robot speed, in meters per second */
        public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        
        /** Max robot rotation rate, in radians per second (not the real one, just the driver limit) */
        public static final double maxAngularRate = 2 * Math.PI;
        
        /** The P value for the rotation PID */
        public static final double rotationP = 8.0;
        /** The I value for the rotation PID */
        public static final double rotationI = 0.0;
        /** The D value for the rotation PID */
        public static final double rotationD = 0.5;
        /** The max angular acceleration */
        public static final double rotationMaxA = maxAngularRate * 5;

        /** Robot speed multiplier on middle of bump */
        public static final double robotBumpSpeed = 0.2;

        /** Changeable speed for speed over bump */
        public static final double variableBumpSpeed = 1.0 - robotBumpSpeed;
    }

    /** Constants for use during Auto */
    public static class AutoConstants {

    }

    /** Robot measurements, in METERS */
    public static class RobotConstants {
        /** X-component (robot centric) offset of the turret center from robot center */
        public static final double turretOffsetX = -0.3;

        /** Y-component (robot centric) offset of the turret center from robot center */
        public static final double turretOffsetY = 0.0;

        /** When the turret is at 0 rotations, how many rotations are added to the robot rotations to get the field-centric turret rotations */
        public static final double turretAddedRotations = 0.0;
    }

    /** Positions of stuff on the field in METERS, Welded field */
    public static class FieldConstants {
        /** Middle of the field between top and bottom, Y coordinate */
        public static final double midLineY = 4.0346376;

        /** Half the width of the hub. Add or subtract from midline to get bump start positions. */
        public static final double hubHalfWidth = 0.6034024;

        /** Distance from hub wall to end of bump */
        public static final double bumpWidth = 1.8542;

        /** Distance from left to right of bump */
        public static final double bumpDepth = 1.12776;

        /** Distance from center of bump to side of bump */
        public static final double bumpHalfDepth = bumpDepth / 2.0;

        /** Distance between center of robot and center of bump where robot starts to slow down */
        public static final double bumpSlowdownDistance = bumpHalfDepth + 0.3;

        /** Distance between center of robot and center of bump where robot starts to turn */
        public static final double bumpTurnDistance = bumpSlowdownDistance + 0.2;

        /** X-coordinate of the center of the red hub */
        public static final double redHubCenterX = 11.915521;

        /** X-coordinate of the center of the blue hub */
        public static final double blueHubCenterX = 4.6255178;
    }

    public static class LookupTableConstants {
        /** Lookup table for distances (in meters) and speeds (in RPS) */
        public static final double[][] distanceSpeedTable = {
            {1.0, 15},
            {1.5, 20},
            {2.0, 25},
            {2.5, 30},
            {3.0, 35},
            {3.5, 40},
            {4.0, 45},
            {4.5, 50},
            {5.0, 55},
            {5.5, 60},
            {6.0, 65},
            {6.5, 70},
            {7.0, 75},
            {7.5, 80},
            {8.0, 85},
            {8.5, 90},
            {9.0, 95},
            {9.5, 100}
        };

        /** Lookup table for distances (in meters) and time of flight (in seconds) */
        public static final double[][] distanceTimeOfFlightTable = {
            {1.0, 1.0},
            {1.5, 1.1},
            {2.0, 1.2},
            {2.5, 1.3},
            {3.0, 1.4},
            {3.5, 1.5},
            {4.0, 1.6},
            {4.5, 1.7},
            {5.0, 1.8},
            {5.5, 1.9},
            {6.0, 2.0},
            {6.5, 2.1},
            {7.0, 2.2},
            {7.5, 2.3},
            {8.0, 2.4},
            {8.5, 2.5},
            {9.0, 2.6},
            {9.5, 2.7},
            {10.0, 2.8}
        };

        /** How many calculation loops to do for the shoot on the move calculation */
        public static final int sotmCalcLoops = 10;

        /** The acceptable error amount for shoot on the move calculations */
        public static final double acceptableError = 0.3;
    }

    /** Constants for vision */
    public static class VisionConstants {
        /** The field tag layout */
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        /** Standard deviations for when only one tag is seen */
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.5);
        /** Standard deviations for when multiple tags are seen */
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 0.2);

        /** Number of cameras on the robot */
        public static final int numCameras = 4;

        /** Camera names */
        public static final String[] cameraNames = {
            "Arducam_OV9281_USB1",
            "Arducam_OV9281_USB2",
            "Arducam_OV9281_USB3",
            "Arducam_OV9281_USB4"
        };

        /** Camera positions on the robot */
        public static final Transform3d[] cameraOffsets = {
            new Transform3d(),
            new Transform3d(),
            new Transform3d(),
            new Transform3d()
        };
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

    }

    /** Constants for LEDs */
    public static class LedConstants {

    }
}
