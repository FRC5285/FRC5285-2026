package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;

public class PositionMath {

    private final Supplier<Pose2d> drivetrainPose;
    private final Supplier<Double> drivetrainVelocityX;
    private final Supplier<Double> drivetrainVelocityY;
    private Rotation2d lastRotation;

    public PositionMath(Supplier<Pose2d> drivetrainPoseSupplier, Supplier<Double> drivetrainVelocityXSupplier, Supplier<Double> drivetrainVelocityYSupplier) {
        this.drivetrainPose = drivetrainPoseSupplier;
        this.drivetrainVelocityX = drivetrainVelocityXSupplier;
        this.drivetrainVelocityY = drivetrainVelocityYSupplier;

        this.lastRotation = this.drivetrainPose.get().getRotation();
    }

    /**
     * The start position of the drivetrain, at enable
     * 
     * @return The drivetrain start position
     */
    public Pose2d drivetrainStartPosition() {
        return new Pose2d(0.0, 0.0, new Rotation2d(0.0));
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
        return Math.min(1.0, OperatorConstants.robotBumpSpeed + Math.min(blueRatio, redRatio) * OperatorConstants.variableBumpSpeed);
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
                * (1.0 - MathUtil.applyDeadband(throttleAmount, OperatorConstants.driveDeadband) * OperatorConstants.throttleMinMultiplier);
    }

    /**
     * Math for modifying driver rotation joystick input to robot
     * 
     * @param controllerInput the controller input for drivetrain rotation speed
     * @param throttleAmount the amount of throttle to apply, 0.0 = max speed, 1.0 = least speed
     * @return the angular velocity output to the drivetrain
     */
    public double driveRotationMath(double controllerInput, double throttleAmount) {
        // resets rotation for autorotation, weird math because it's stupid
        this.lastRotation = new Rotation2d(this.drivetrainPose.get().getRotation().getRadians() - Math.PI);

        // invert controllerInput (because the default controller direction is stupid)
        return MathUtil.applyDeadband(-controllerInput, OperatorConstants.driveDeadband)
                * this.driveSpeedMultiplier() * OperatorConstants.maxAngularRate
                * (1.0 - MathUtil.applyDeadband(throttleAmount, OperatorConstants.driveDeadband) * OperatorConstants.throttleMinMultiplier);
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
            this.lastRotation = new Rotation2d(-this.drivetrainVelocityX.get(), -this.drivetrainVelocityY.get());
        }
        return this.lastRotation;
    }

    /**
     * The target speed for the turret flywheel, in rotations per second
     * 
     * @return The target flywheel speed, in rotations per second
     */
    public double getFlywheelSpeedTarget() {
        return 0.0;
    }

    /**
     * The target rotation for the turret
     * 
     * @return The turret rotation target, in radians
     */
    public double getTurretRotationTarget() {
        return 0.0;
    }
}
