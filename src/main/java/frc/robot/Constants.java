package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.generated.TunerConstants;

public final class Constants {
    /** Constants relating to the driver input */
    public static class OperatorConstants {
        /** The driver station port to use as the driver controller port */
        public static final int driverControllerPort = 0;

        /** The port for the second controller */
        public static final int secondControllerPort = 1;

        /** The deadband for the driver controller joysticks */
        public static final double driveDeadband = 0.1;

        /** Minimum speed multiplier for throttle */
        public static final double throttleMinMultiplier = 0.1;

        /** Max robot speed, in meters per second */
        public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        
        /** Max robot rotation rate, in radians per second (not the real one, just the driver limit) */
        public static final double maxAngularRate = 2 * Math.PI;

        /** The P value for the auton drive PID */
        public static final double driveP = 10.0;
        /** The I value for the auton drive PID */
        public static final double driveI = 0.0;
        /** The D value for the auton drive PID */
        public static final double driveD = 0.0;
        
        /** The P value for the rotation PID */
        public static final double rotationP = 5.0;
        /** The I value for the rotation PID */
        public static final double rotationI = 0.0;
        /** The D value for the rotation PID */
        public static final double rotationD = 0.0;
        /** The max angular acceleration */
        public static final double rotationMaxA = maxAngularRate * 5;

        public static final double pidDistanceTolerance = 0.02;

        /** Robot speed multiplier on middle of bump */
        public static final double robotBumpSpeed = 0.15;

        /** Robot speed multiplier when shooting */
        public static final double shootVelocityMultiplier = 0.2;

        /** Changeable speed for speed over bump */
        public static final double variableBumpSpeed = 1.0 - robotBumpSpeed;
    }

    /** Constants for use during Auto */
    public static class AutoConstants {
        /** Max velocity during auton */
        public static final double maxV = 3.0;
        
        /** Max acceleration during auton */
        public static final double maxA = 3.0;

        /** Max angular velocity during auton, in radians */
        public static final double maxAngularV = 2 * Math.PI;

        /** Max angular acceleration during auton, in radians */
        public static final double maxAngularA = 6 * Math.PI;

        /** Max velocity during climb auton */
        public static final double climbMaxV = 3.0;
        
        /** Max acceleration during climb auton */
        public static final double climbMaxA = 3.0;

        /** Max angular velocity during climb auton, in radians */
        public static final double climbMaxAngularV = 2 * Math.PI;

        /** Max angular acceleration during climb auton, in radians */
        public static final double climbMaxAngularA = 6 * Math.PI;

        /** Max seconds to finetune the climb */
        public static final double fineTuneMaxTime = 3.0;

        /** How long to wait at the outpost */
        public static final double outpostWaitTime = 3.0;

        /** How long to shoot */
        public static final double shootTime = 5.0;

        /** How long to shoot when there is more fuel */
        public static final double shootMoreTime = 6.0;

        /** Max time to wait for turret intake to get up to speed */
        public static final double turretIntakeMaxWaitTime = 1.0;
    }

    /** Robot measurements, in METERS */
    public static class RobotConstants {
        /** X-component (robot centric) offset of the turret center from robot center */
        public static final double turretOffsetX = -0.3;

        /** Y-component (robot centric) offset of the turret center from robot center */
        public static final double turretOffsetY = 0.0;

        /** When the turret is at 0 rotations, how many rotations are added to the robot rotations to get the field-centric turret rotations */
        public static final double turretAddedRotations = 0.50;

        /** Width of the robot WITHOUT BUMPER, in meters */
        public static final double robotWidth = 0.6858;

        /** Width of the robot WITH BUMPER, in meters */
        public static final double robotWidthBumpers = 0.8382;

        /** Offset of the hook beginning, X-component, robot-centric */
        public static final double robotHookOffsetXBeginning = -0.1397;

        /** Offset of the hook, X-component, robot-centric */
        public static final double robotHookOffsetX = -0.0762;

        /** Offset of the hook, Y-component, robot-centric */
        public static final double robotHookOffsetY = -robotWidth / 2.0;
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
        public static final double bumpSlowdownDistance = bumpHalfDepth + 0.5;

        /** Distance between center of robot and center of bump where robot starts to turn */
        public static final double bumpTurnDistance = bumpSlowdownDistance + 0.0;

        /** X-coordinate of the center of the red hub */
        public static final double redHubCenterX = 11.915521;

        /** X-coordinate of the center of the blue hub */
        public static final double blueHubCenterX = 4.6255178;

        /** X-coordinate of the blue starting positions */
        public static final double blueStartX = blueHubCenterX - hubHalfWidth - (RobotConstants.robotWidthBumpers / 2.0);

        /** X-coordinate of the red starting positions */
        public static final double redStartX = redHubCenterX + hubHalfWidth + (RobotConstants.robotWidthBumpers / 2.0);

        /** Y-component of the translation between the middle Y and the Y coordinate of the centers of the bumps */
        public static final double startTranslationY1 = hubHalfWidth + (bumpWidth / 2.0);

        /** Half of the Y-component of the translation between the middle Y and the Y coordinate where the robot corner is aligned with the trench side wall closest to the bump */
        public static final double startTranslationY2Half = 3.165 / 2.0;

        /** Pose of the center apriltag on the blue outpost */
        public static final Pose2d blueOutpostTag = new Pose2d(0.0077469999999999995, 0.6659626, new Rotation2d());

        /** Distance between the bumper and the outpost when parking during auton */
        public static final double bumperOutpostDistance = 0.3;

        /** Pose of the robot when parking in front of the outpost during auton */
        public static final Pose2d blueOutpostParkPose = new Pose2d(blueOutpostTag.getX() + bumperOutpostDistance + (RobotConstants.robotWidthBumpers / 2.0), blueOutpostTag.getY(), new Rotation2d(Math.PI));

        /** Pose of the robot when shooting after getting fuel from the blue outpost */
        public static final Pose2d blueOutpostShootPose = new Pose2d(2.0, 2.5, new Rotation2d(Math.PI));

        /** The pose between the apriltags on the blue tower */
        public static final Pose2d blueTowerTags = new Pose2d(0.0080772, 3.9616126, new Rotation2d());

        /** The pose between the apriltags on the red tower */
        public static final Pose2d redTowerTags = new Pose2d(16.5329616, 4.1076626, new Rotation2d());

        /** How much to offset the hook position from the closest possible position to the tower */
        public static final double hookOffset = 0.0127;

        /** Pose of the climb latch position (where it hooks onto) for the right side of the tower, blue alliance - flipped automatically by pathplanner */
        public static final Pose2d towerRightLatchPose = new Pose2d(blueTowerTags.getX() + 1.06108, 3.7457125999999996 - (0.479425 + hookOffset), new Rotation2d());

        /** Pose of the climb latch position (where it hooks onto) for the left side of the tower, blue alliance - flipped automatically by pathplanner */
        public static final Pose2d towerLeftLatchPose = new Pose2d(blueTowerTags.getX() + 1.06108, 3.7457125999999996 + (0.479425 + hookOffset), new Rotation2d());

        /** Pose of the robot during the climb, right side of the blue tower, flipped automatically by pathplanner */
        public static final Pose2d towerRightFinalPose = new Pose2d(towerRightLatchPose.getX() + RobotConstants.robotHookOffsetX, towerRightLatchPose.getY() + RobotConstants.robotHookOffsetY, new Rotation2d(Math.PI));

        /** Pose of the robot during the climb, left side of the blue tower, flipped automatically by pathplanner */
        public static final Pose2d towerLeftFinalPose = new Pose2d(towerLeftLatchPose.getX() - RobotConstants.robotHookOffsetX, towerLeftLatchPose.getY() - RobotConstants.robotHookOffsetY, new Rotation2d(0.0));

        /** Field-centric X translation before the robot climbs */
        public static final double climbPrepX = 0.1016;

        /** Field-centric Y translation before the robot climbs */
        public static final double climbPrepY = 0.0762;

        /** Pose of the robot before the climb, right of the blue tower, flipped automatically by pathplanner */
        public static final Pose2d towerRightPrepPose = new Pose2d(towerRightFinalPose.getX() - climbPrepX, towerRightFinalPose.getY() - climbPrepY, towerRightFinalPose.getRotation());

        /** Pose of the robot before the climb, left of the blue tower, flipped automatically by pathplanner */
        public static final Pose2d towerLeftPrepPose = new Pose2d(towerLeftFinalPose.getX() + climbPrepX, towerLeftFinalPose.getY() + climbPrepY, towerLeftFinalPose.getRotation());
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

        /** Add time to the SOTM calculation */
        public static final double addedDelay = 0.02;
    }

    /** Constants for vision */
    public static class VisionConstants {
        /** The field tag layout */
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        /** Standard deviations for when only one tag is seen */
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.5);
        /** Standard deviations for when multiple tags are seen */
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 0.2);

        /** The time for the camera transform buffer */
        public static final double camPositionBufferTime = 2.0;

        /** Number of cameras on the robot */
        public static final int numCameras = 0;

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
            new Transform3d(new Translation3d(), new Rotation3d(new Rotation2d(-Math.PI / 4.0))),
            new Transform3d(new Translation3d(), new Rotation3d(new Rotation2d(Math.PI / 4.0)))
        };

        /** If a camera is mounted on the turret - the camera offset should be relative to the turret center when turret rotation is 0 if true */
        public static final boolean[] turretMounted = {
            true,
            true,
            false,
            false
        };
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
        public static final int motorCanId = 18;

        // arbitrary values (for now)
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 4.8;
        public static final double kI = 0;
        public static final double kD = 0.1;

        // motion magic settings
        public static final double cruiseVelocity = 80;
        public static final double acceleration = 320;
        public static final double jerk = 1600;

        public static final double intakeSpeed = 160.0;
        public static final double reverseSpeed = -10.0;

        public static final double speedTolerance = 16.7;
    }

    /** Constants for storage rollers */
    public static class RollerConstants {
        public static final int ROLLER_MOTOR_ID=19;
        public static final double speed=0.4;
        public static final double fastSpeed = 0.8;
    }

    /** Constants for climber */
    public static class ClimbConstants {

    }

    /** Constants for LEDs */
    public static class LEDConstants {
        public static final int led_pin = 0;

        /** The seconds before the hub activates where the leds do the warning lights */
        public static final double shiftAlmostNum = 3.0;

        public static final double hotPink = 0.57; //trans flag colours
        public static final double darkred = 0.59;
        public static final double red = 0.61; //resevered for errors/ warnings
        public static final double redOrange = 0.63;
        public static final double orange = 0.65;
        public static final double gold = 0.67;
        public static final double yellow = 0.69;
        public static final double lawnGreen = 0.71;
        public static final double lime = 0.73;
        public static final double darkGreen = 0.75;
        public static final double green = 0.77;
        public static final double blueGreen = 0.79;
        public static final double aqua = 0.81;
        public static final double skyBlue = 0.83;
        public static final double darkBlue = 0.85;
        public static final double blue = 0.87;
        public static final double blueViolet = 0.89;
        public static final double violet = 0.91;
        public static final double white = 0.93;
        public static final double gray = 0.95;
        public static final double darkGray = 0.97;
        public static final double black = 0.99;

        

        public static final double rainbow_rainbow = -0.99; //rainbow pattern and rainbow colours

        public static final double off = 0.99;

        public static final double warning = 0.61;

        public static final double[] warning_blink = {
        4.0, warning, off, warning, off
        };

        public static final double[] blink_orange = {
            2.0, orange, off, orange, off
        };

        //pride flag presets
        public static final double[] trans_flag = {
        2.0, skyBlue, hotPink, white, hotPink, skyBlue, off, off
        };

        public static final double[] aroace_flag = {
        2.0, orange, gold, white, skyBlue, darkBlue, off, off
        };

        public static final double[] lesbian_flag = {
        2.0, redOrange, orange, white, hotPink, violet, off, off
        };
        
        public static final double[] sapphic_flag = {
        2.0, hotPink, white, hotPink, off, off
        };

        public static final double[] queer_flag = {
        2.0, red, orange, yellow, lime, blue, violet, off, off, off
        };

        public static final double[] lithrosexual_flag = {
        2.0, redOrange, orange, yellow, white, black, off, off
        };

        public static final double[] femaric_flag = {
        2.0, black, white, hotPink, off, off
        };

        public static final double[] mascic_flag = {
        2.0, black, white, skyBlue, off, off
        };

        public static final double[] genderfluid_flag = {
        2.0, hotPink, white, violet, black, blueViolet, off, off
        };

        public static final double[] achillean_flag = {
        2.0, skyBlue, white, skyBlue, off, off
        };

        public static final double[] genderqueer_flag = {
        2.0, violet, white, lime, off, off
        };
        
        public static final double[] aro_flag = {
        2.0, darkGreen, lime, white, gray, black, off, off
        };

        public static final double[] ace_flag = {
        2.0, black, gray, white, violet, off, off
        };

        public static final double[] oriented_aroace = {
        2.0, black, gray, white, blueGreen, off, off
        };

        public static final double[] enby_flag = {
        2.0, yellow, white, violet, black, off, off
        };

        public static final double[] neutrois_flag = {
        2.0, white, lime, black, off, off
        };

        public static final double[] androgyne_flag = {
        2.0, hotPink, violet, skyBlue, off, off
        };

        public static final double[] polyamorous_flag = {
        2.0, blue, red, black, off, off
        };

        public static final double[] transfem_flag = {
        2.0, skyBlue, white, hotPink, white, skyBlue, off, off
        };

        public static final double[] bisexual_flag = {
        2.0, hotPink, violet, blueViolet, off, off
        };

        public static final double[] agender_flag = {
        2.0, black, gray, white, lime, gray, black, off, off
        };

        public static final double[] diamoric_flag = {
        2.0, lime, white, lime, off, off
        };

        public static final double[] libramasc_flag = {
        2.0, black, gray, white, skyBlue, white, gray, black, off, off
        };

        public static final double[] librafemme_flag = {
        2.0, black, gray, white, violet, white, gray, black, off, off
        };

        public static final double[] polygender_flag = {
        2.0, black, gray, blue, yellow, hotPink, off,
        };

        public static final double[] queerplatonic_flag = {
        2.0, yellow, hotPink, white, gray, black, off, off
        };

        public static final double[] pansexual_flag = {
        2.0, hotPink, yellow, skyBlue, off, off
        };

        public static final double[] maverique_flag = { 
        2.0, yellow, white, orange, off, off
        };

        public static final double[] greysexual_flag = {
        2.0, violet, gray, white, gray, violet, off, off
        };

        public static final double[] polysexual_flag = {
        2.0, hotPink, lime, skyBlue, off, off
        };

        public static final double[] trigender_flag = {
        2.0, hotPink, violet, lime, violet, hotPink, off, off
        };

        public static final double[] greyromantic_flag = {  
        2.0, darkGreen, gray, white, gray, darkGreen, off, off
        };
    }
}
