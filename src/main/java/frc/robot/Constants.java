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
        public static final double maxAngularRate = 4 * Math.PI;

        /** Gets to max speed in 1/driverAccelLimit seconds */
        public static final double driverAccelLimit = 2.0;

        /** Gets to max rotation speed in 1/driverRotAccelLimit seconds */
        public static final double driverRotAccelLimit = 2.0;

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
        public static final double shootVelocityMultiplier = 0.5;

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
        public static final double shootTime = 10.0;

        /** How long to shoot when there is more fuel */
        public static final double shootMoreTime = 10.0;

        /** Max time to wait for turret intake to get up to speed */
        public static final double turretIntakeMaxWaitTime = 1.0;
    }

    /** Robot measurements, in METERS */
    public static class RobotConstants {
        /** X-component (robot centric) offset of the turret center from robot center */
        public static final double turretOffsetX = -0.1840732628;

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
        public static final Pose2d blueOutpostShootPose = new Pose2d(2.5, 2.5, new Rotation2d(-3.0 * Math.PI / 4.0));

        public static final Pose2d blueLeftShootPose = new Pose2d(2.5, 5.5, new Rotation2d(3.0 * Math.PI / 4.0));

        public static final Pose2d blueRightShootPose = new Pose2d(2.5, 2.5, new Rotation2d(-3.0 * Math.PI / 4.0));

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
        public static final double climbPrepY = 0.3048;

        /** Pose of the robot before the climb, right of the blue tower, flipped automatically by pathplanner */
        public static final Pose2d towerRightPrepPose = new Pose2d(towerRightFinalPose.getX() - climbPrepX, towerRightFinalPose.getY() - climbPrepY, towerRightFinalPose.getRotation());

        /** Pose of the robot before the climb, left of the blue tower, flipped automatically by pathplanner */
        public static final Pose2d towerLeftPrepPose = new Pose2d(towerLeftFinalPose.getX() + climbPrepX, towerLeftFinalPose.getY() + climbPrepY, towerLeftFinalPose.getRotation());
    }

    public static class LookupTableConstants {
        /** Lookup table for distances (in meters) and speeds (in RPS) */
        public static final double[][] distanceSpeedTable = {
            {1.47, 42},
            {2.01, 41.5},
            {2.84, 47},
            {3.66, 54.5},
            {4.80, 58},
            {5.82, 67},
            {7.09, 75},
            {7.87, 88},
            {9.09, 100}
        };

        /** Lookup table for distances (in meters) and time of flight (in seconds) */
        public static final double[][] distanceTimeOfFlightTable = {
            {1.47, 0.96},
            {2.01, 0.81},
            {2.84, 0.98},
            {3.66, 1.18},
            {4.80, 1.24},
            {5.82, 1.51},
            {7.09, 1.68},
            {7.87, 1.88},
            {9.09, 2.04}
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
            new Transform3d(-0.28807, -0.30565, 0.19173, new Rotation3d(0.0, -14.04 * (Math.PI / 180.0), -30.3 * (Math.PI / 180.0))),
            new Transform3d(0.16908 - 0.17455, 0.12833, 0.59485 + 0.12790, new Rotation3d(0.0, 0.0, 15.0 * (Math.PI / 180.0))),
            new Transform3d(-0.28807, 0.30565, 0.19173, new Rotation3d(0.0, -14.04 * (Math.PI / 180.0), 30.3 * (Math.PI / 180.0))), // cam 3 on opposite side of cam 1
            new Transform3d(0.17580 - 0.17455, -0.12002, 0.61376 + 0.12790, new Rotation3d(0.0, -17.0 * (Math.PI / 180.0), 0.0))
        };

        /** If a camera is mounted on the turret - the camera offset should be relative to the turret center when turret rotation is 0 if true */
        public static final boolean[] turretMounted = {
            false,
            true,
            false,
            true
        };
    }

    /** Constants for turret */
    public static class TurretConstants {
        public static final double kS = 0.0; //turret base
        public static final double kV = 11.2;
        public static final double kp = 80.0;
        public static final double ki = 0.0;
        public static final double kd = 0.0;
        public static final double turretMaxV = 2.0;
        public static final double turretMaxA = 2.0;
        
        public static final double S_ACceleration = 320;
        public static final double S_Jerk = 420.69;

        public static final double S_kS = 0.0; //shooter
        public static final double S_kV = 0.12;
        public static final double S_kA = 0.0;
        public static final double S_kp = 0.1;
        public static final double S_ki = 0.0;
        public static final double S_kd = 0.0;
        /** The CAN ID for the motor */
        public static final int motorCanId = 15;
        public static final int ShooterMotorCanId = 16;
        public static final int ShooterMotor2CanId = 17;

        public static final int channel_a = 0;
        public static final int channel_b = 1;
 
        public static final double encoderAOffset = -0.487;
        public static final double encoderBOffset = -0.111;
        public static final double max_range = 0.5; //rotations
        public static final double min_range = -0.5; //rotaitons
        public static final double match_tolerance = 0.03; //rotations

        public static final double turretPIDMin = -0.25;
        public static final double turretPIDMax = 0.25;

        public static final double shooterMaxSpeed = 100.0;
        public static final double shooterMinSpeed = 0.0;

        /** Tolerance of turret PID, in rotations - used to determine if shooter should shoot */
        public static final double turretTolerance = 0.03;
    }

    /** Constants for intake */
    public static class IntakeConstants {
        public static final int intakeID = 21;
        public static final int lowerID = 22;
        public static final int followerId = 24;
        public static final int encoderChannel = 2;
        
        /** Encoder value when it is supposed to be at 0 (0 is when the intake is flat) */
        public static final double intakeLoweredValue = 2.0; // there are a few wires stopping the intake from being fully lowered
        public static final double intakeRaisedValue = 0.0; // flimsy build so its variable

        public static final double kS = 0.0;
        public static final double kV = 0.0; // 3.5
        public static final double kP = 0.0; // 1.0
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double maxVel = 4.0; // for trapezoidal profile; max velocity in rotations/second
        public static final double maxAcc = 2.0; // max acceleration in rotations/second/second

        // intake settings
        public static final double intakeVolts = -12.0; // 12.0

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
        public static final int ROLLER_MOTOR_ID=19;
        public static final double speed=0.0;
        public static final double fastSpeed = 1.0; // 0.2
    }

    /** Constants for the bucket outtake */
    public static class BucketOutConstants {
        public static final int MOTOR_ID = 20;
        public static final double SPEEDForwards = -1.0; // adjust as needed
        public static final double SPEEDBackwards = 0.4;
        public static final double forwardSeconds = 5.5;
        public static final double backwardSeconds = 0.0;
    }
        
    

    /** Constants for climber */
    public static class ClimbConstants {
    
        //**************CLIMB MOTOR CONSTANTS************
        public static final int climbMotorID = 23;

        public static final double ckP = 40.0; //need to calibrate
        public static final double ckI = 0.0; //need to calibrate
        public static final double ckD = 0.0; //need to calibrate

        public static final double cmaxV = 1.5; //need to calibrate
        public static final double cmaxA = 0.5; //need to calibrate


        //***************ROTATE MOTOR CONSTANTS***************
        public static final int rotateMotorID = 24;

        public static final int encoderChannel = 3;

        public static final double encoderStartRotations = 0.867 - 0.3;
        public static final double rotateInitialRotations = 0.3; //need to calibrate
        public static final double rotateGoalRotations = 0.8; //need to calibrate

        public static final double rkP = 36.0; //need to calibrate
        public static final double rkI = 0.0; //need to calibrate
        public static final double rkD = 0.0; //need to calibrate

        public static final double rmaxV = 4.5; //need to calibrate
        public static final double rmaxA = 3.5; //need to calibrate

        
        //**************LIDAR CONSTANTS************************
        public static final double maxExtension = 0.21; //distance between sensor and plate (meters)

        public static final double middleExtension = 0.06; // auton climb distance

        public static final double minExtension = 0.045; //distance between sensor and plate (meters)

        public static final double lidarOffset = 0.0; // Distance of lidar measurement - real measurement // 0.025

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
