package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LookupTableConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TurretConstants;

public class PositionMath {

    private final LookupTable speedTable;
    private final LookupTable timeOfFlightTable;

    private Supplier<Pose2d> drivetrainPose;
    private Supplier<Double> drivetrainVelocityX;
    private Supplier<Double> drivetrainVelocityY;
    private Supplier<Double> drivetrainVelocityRotation;
    private Supplier<Double> turretRotation;
    private Supplier<Boolean> robotShooting;
    private Supplier<Boolean> climbing;

    private Alliance robotAlliance;
    private Rotation2d lastRotation;
    private Pose2d hubPose;
    private Translation2d shootVector;

    private Pose2d lastCalcPose;

    public PositionMath() {
        this.speedTable = new LookupTable(LookupTableConstants.distanceSpeedTable, "Speed Table");
        this.timeOfFlightTable = new LookupTable(LookupTableConstants.distanceTimeOfFlightTable, "Time of Flight Table");

        this.resetSide();

        // Placeholders
        this.drivetrainPose = () -> this.drivetrainStartPosition();
        this.drivetrainVelocityX = () -> 0.0;
        this.drivetrainVelocityY = () -> 0.0;
        this.drivetrainVelocityRotation = () -> 0.0;
        this.turretRotation = () -> 0.0;
        this.robotShooting = () -> false;
        this.climbing = () -> false;

        // Set previous drivetrain rotation target
        this.resetLastRotation();

        this.lastCalcPose = new Pose2d();
        this.calculateShootVector();
    }

    /**
     * Sets the suppliers
     * 
     * @param drivetrainPoseSupplier Supplier for the drivetrain pose
     * @param drivetrainVelocityXSupplier Supplier for the drivetrain Field-Centric X velocity
     * @param drivetrainVelocityYSupplier Supplier for the drivetrain Field-Centric Y velocity
     * @param drivetrainVelocityRotationSupplier Supplier for the drivetrain angular velocity
     * @param turretRotationSupplier Supplier for the raw turret rotation, in rotations
     * @param shootingSupplier Supplier for if the robot is trying to shoot
     * @param climbingSupplier Supplier for if the robot is climbing
     */
    public void setSuppliers(
        Supplier<Pose2d> drivetrainPoseSupplier,
        Supplier<Double> drivetrainVelocityXSupplier,
        Supplier<Double> drivetrainVelocityYSupplier,
        Supplier<Double> drivetrainVelocityRotationSupplier,
        Supplier<Double> turretRotationSupplier,
        Supplier<Boolean> shootingSupplier,
        Supplier<Boolean> climbingSupplier
    ) {
        // Set suppliers
        this.drivetrainPose = drivetrainPoseSupplier;
        this.drivetrainVelocityX = drivetrainVelocityXSupplier;
        this.drivetrainVelocityY = drivetrainVelocityYSupplier;
        this.drivetrainVelocityRotation = drivetrainVelocityRotationSupplier;
        this.turretRotation = turretRotationSupplier;
        this.robotShooting = shootingSupplier;
        this.climbing = climbingSupplier;

        this.resetSide();
        this.resetLastRotation();
    }

    /**
     * The start position of the drivetrain, at enable, on the middle starting position
     * 
     * @return The drivetrain start position
     */
    public Pose2d drivetrainStartPosition() {
        return this.drivetrainStartPosition(0);
    }

    /**
     * The start position of the drivetrain, intake facing towards the drivers
     * 
     * @param position the start position in the range [-2, 2], -2 representing the leftmost and 2 representing the rightmost
     * @return the Pose2d representation
     */
    public Pose2d drivetrainStartPosition(int position) {
        Rotation2d robotRotation;
        double robotX;
        double robotY = 0.0;

        // Start position Y translation
        if (Math.abs(position) == 1) {
            robotY = FieldConstants.startTranslationY1 * position;
        } else if (Math.abs(position) == 2) {
            robotY = FieldConstants.startTranslationY2Half * position;
        }

        // Side specific transformations
        if (this.robotAlliance == Alliance.Blue) {
            robotRotation = new Rotation2d(Math.PI);
            robotX = FieldConstants.blueStartX;
            robotY = -robotY;
        } else {
            robotRotation = new Rotation2d(0);
            robotX = FieldConstants.redStartX;
        }

        return new Pose2d(robotX, FieldConstants.midLineY + robotY, robotRotation);
    }

    /**
     * The drivetrain speed multiplier. Used when going over the bump.
     * 
     * @return The drivetrain speed multiplier
     */
    public double driveSpeedMultiplier() {
        double robotX = this.drivetrainPose.get().getX();
        double blueRatio = Math.abs(FieldConstants.blueHubCenterX - robotX) / FieldConstants.bumpSlowdownDistance;
        double redRatio = Math.abs(FieldConstants.redHubCenterX - robotX) / FieldConstants.bumpSlowdownDistance;
        double shootMult = this.robotShooting.get() ? OperatorConstants.shootVelocityMultiplier : 1.0;
        return Math.min(shootMult, OperatorConstants.robotBumpSpeed + Math.min(blueRatio, redRatio) * OperatorConstants.variableBumpSpeed);
    }

    /**
     * If the robot is close enough to the bump to turn
     * 
     * @return whether to do the bump turn
     */
    public boolean bumpTurn() {
        double robotX = this.drivetrainPose.get().getX();
        double blueDist = Math.abs(FieldConstants.blueHubCenterX - robotX);
        double redDist = Math.abs(FieldConstants.redHubCenterX - robotX);
        return Math.min(redDist, blueDist) <= FieldConstants.bumpTurnDistance;
    }

    /**
     * Math for modifying driver joystick input to robot
     * 
     * @param controllerInput the controller input for drivetrain x/y movement
     * @param throttleAmount the amount of throttle to apply, 0.0 = max speed, 1.0 = least speed
     * @return the velocity output to the drivetrain
     */
    public double driveJoystickMath(double controllerInput, double throttleAmount) {
        // invert controllerInput (because the default controller direction is stupid)
        return MathUtil.applyDeadband(-controllerInput, OperatorConstants.driveDeadband)
                * this.driveSpeedMultiplier() * OperatorConstants.maxSpeed
                * (1.0 - MathUtil.applyDeadband(throttleAmount, OperatorConstants.driveDeadband) * (1.0 - OperatorConstants.throttleMinMultiplier));
    }

    /**
     * Math for modifying driver rotation joystick input to robot
     * 
     * @param controllerInput the controller input for drivetrain rotation speed
     * @param throttleAmount the amount of throttle to apply, 0.0 = max speed, 1.0 = least speed
     * @return the angular velocity output to the drivetrain
     */
    public double driveRotationMath(double controllerInput, double throttleAmount) {
        this.resetLastRotation();

        // invert controllerInput (because the default controller direction is stupid)
        return MathUtil.applyDeadband(-controllerInput, OperatorConstants.driveDeadband)
                * this.driveSpeedMultiplier() * OperatorConstants.maxAngularRate
                * (1.0 - MathUtil.applyDeadband(throttleAmount, OperatorConstants.driveDeadband) * (1.0 - OperatorConstants.throttleMinMultiplier));
    }

    /**
     * The drivetrain rotation amount
     * 
     * @param controllerInput the controller input for drivetrain rotation
     * @return the rotation rate for drivetrain rotation
     */
    public Rotation2d drivetrainRotationAmount() {
        if (this.bumpTurn()) {
            this.lastRotation = new Rotation2d((Math.floor(Math.abs(this.lastRotation.getRadians()) / (Math.PI / 2)) * (Math.PI / 2) + (Math.PI / 4)) * Math.signum(this.lastRotation.getRadians()));
        } else if (Math.abs(this.drivetrainVelocityX.get()) + Math.abs(this.drivetrainVelocityY.get()) > 0.8) {
            this.lastRotation = new Rotation2d(this.drivetrainVelocityX.get(), this.drivetrainVelocityY.get());
            if (this.robotAlliance == Alliance.Red) {
                this.lastRotation = new Rotation2d(-this.lastRotation.getCos(), -this.lastRotation.getSin());
            }
        }
        return this.lastRotation;
    }

    /**
     * The target speed for the turret flywheel, in rotations per second
     * 
     * @return The target flywheel speed, in rotations per second
     */
    public double getFlywheelSpeedTarget() {
        // Save compute
        if (!this.lastCalcPose.equals(this.drivetrainPose.get())) {
            this.calculateShootVector();
        }

        if (!this.robotShooting.get()) {
            return 0.0;
        }

        double dist = this.shootVector.getNorm();

        return this.speedTable.getOutput(dist);
    }

    /**
     * If shooting is a good idea
     * 
     * @return if the error for the shoot on the move calculation is smaller than acceptable and the turret rotation is good
     */
    public boolean shouldShoot() {
        return this.timeOfFlightTable.getCalcError() < LookupTableConstants.acceptableError;
        // && Math.abs(this.turretRotation.get() * 2.0 * Math.PI - this.getTurretRotationTarget()) < TurretConstants.turretTolerance;
    }

    /**
     * The pose of the turret, Field-Centric
     * 
     * @return the turret pose
     */
    public Pose2d getTurretPose() {
        Pose2d currentPose = this.drivetrainPose.get();
        return new Pose2d(
            currentPose.getX() + currentPose.getRotation().getCos() * RobotConstants.turretOffsetX - currentPose.getRotation().getSin() * RobotConstants.turretOffsetY,
            currentPose.getY() + currentPose.getRotation().getSin() * RobotConstants.turretOffsetX + currentPose.getRotation().getCos() * RobotConstants.turretOffsetY,
            this.getTurretRotation()
        );
    }

    /**
     * The current rotation of the turret, Field-Centric
     * 
     * @return the current turret rotation
     */
    public Rotation2d getTurretRotation() {
        double currentRotation = this.drivetrainPose.get().getRotation().getRotations();
        double r = currentRotation + RobotConstants.turretAddedRotations + this.turretRotation.get();

        return new Rotation2d(r * 2 * Math.PI);
    }

    /** The turret X velocity, Field-Centric */
    public double getTurretXVelocity() {
        Translation2d turretPoseDiff = this.getTurretPose().getTranslation().minus(this.drivetrainPose.get().getTranslation());
        return this.drivetrainVelocityX.get() - turretPoseDiff.getY() * this.drivetrainVelocityRotation.get();
    }

    /** The turret Y velocity, Field-Centric */
    public double getTurretYVelocity() {
        Translation2d turretPoseDiff = this.getTurretPose().getTranslation().minus(this.drivetrainPose.get().getTranslation());
        return this.drivetrainVelocityY.get() + turretPoseDiff.getX() * this.drivetrainVelocityRotation.get();
    }

    /** The field-centric turret velocity as a vector represented by a Translation2d */
    public Translation2d getTurretVelocityVector() {
        Translation2d turretPoseDiff = this.getTurretPose().getTranslation().minus(this.drivetrainPose.get().getTranslation());
        double rotationVelocity = this.drivetrainVelocityRotation.get();
        return new Translation2d(
            this.drivetrainVelocityX.get() - turretPoseDiff.getY() * rotationVelocity,
            this.drivetrainVelocityY.get() + turretPoseDiff.getX() * rotationVelocity
        );
    }

    /**
     * Position of camera on the turret, robot-centric
     * 
     * @param cameraTransform the transform of the camera relative to the turret center
     * @return the robot-centric transform of the camera
     */
    public Transform3d getCameraTurretTransform(Transform3d cameraTransform) {
        return new Transform3d(new Transform2d(RobotConstants.turretOffsetX, RobotConstants.turretOffsetY, new Rotation2d((RobotConstants.turretAddedRotations + this.turretRotation.get()) * 2 * Math.PI))).plus(cameraTransform);
    }

    /**
     * Whether the robot is in its alliance zone
     * 
     * @return Whether the robot is in its alliance zone
     */
    public boolean inAllianceZone() {
        if (this.robotAlliance == Alliance.Blue && this.drivetrainPose.get().getX() < FieldConstants.blueHubCenterX) {
            return true;
        }
        if (this.robotAlliance == Alliance.Red && this.drivetrainPose.get().getX() > FieldConstants.redHubCenterX) {
            return true;
        }
        return false;
    }

    /**
     * The X coordinate of the middle of the hub
     * 
     * @return the X coordinate
     */
    public double getAllianceLineX() {
        if (this.robotAlliance == Alliance.Blue) {
            return FieldConstants.blueHubCenterX;
        }
        return FieldConstants.redHubCenterX;
    }

    /** Resets the robot alliance according to driver station data. Runs whenever robot is enabled. */
    public void resetSide() {
        this.robotAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        this.hubPose = new Pose2d(this.getAllianceLineX(), FieldConstants.midLineY, new Rotation2d());
    }

    /** Resets the last stored robot rotation */
    public void resetLastRotation() {
        // resets rotation for autorotation, weird math because it's stupid
        if (this.robotAlliance == Alliance.Blue) {
            this.lastRotation = new Rotation2d(this.drivetrainPose.get().getRotation().getRadians());
        } else {
            this.lastRotation = new Rotation2d(this.drivetrainPose.get().getRotation().getRadians() - Math.PI);
        }
    }

    /**
     * The target rotation for the turret
     * 
     * @return The turret rotation target, in radians
     */
    public double getTurretRotationTarget() {
        if (this.climbing.get()) {
            Translation2d toTags;
            if (this.robotAlliance == Alliance.Blue) {
                toTags = FieldConstants.blueTowerTags.getTranslation().minus(this.drivetrainPose.get().getTranslation());
            } else {
                toTags = FieldConstants.redTowerTags.getTranslation().minus(this.drivetrainPose.get().getTranslation());
            }

            double currentRotation = this.drivetrainPose.get().getRotation().getRotations();
            double r = toTags.getAngle().getRotations() - (currentRotation + RobotConstants.turretAddedRotations);

            return r * 2 * Math.PI;
        }

        // Save compute
        if (!this.lastCalcPose.equals(this.drivetrainPose.get())) {
            this.calculateShootVector();
        }

        double currentRotation = this.drivetrainPose.get().getRotation().getRotations();
        double r = this.shootVector.getAngle().getRotations() - (currentRotation + RobotConstants.turretAddedRotations);

        return r * 2 * Math.PI;
    }

    public Translation2d calculateShootVector() {
        this.lastCalcPose = this.drivetrainPose.get();
        Translation2d dRH;

        if (this.inAllianceZone()) {
            dRH = this.hubPose.getTranslation().minus(this.drivetrainPose.get().getTranslation());
        } else {
            dRH = new Translation2d(this.getAllianceLineX() - this.lastCalcPose.getX(), 0.0);
        }

        this.shootVector = this.timeOfFlightTable.sotmCalc2(this.getTurretVelocityVector(), dRH);

        return this.shootVector;
    }
}
