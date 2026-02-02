package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.OperatorConstants;

public class PositionMath {

    private final Supplier<Pose2d> drivetrainPose;
    private final Supplier<Double> drivetrainVelocityX;
    private final Supplier<Double> drivetrainVelocityY;

    public PositionMath(Supplier<Pose2d> drivetrainPoseSupplier, Supplier<Double> drivetrainVelocityXSupplier, Supplier<Double> drivetrainVelocityYSupplier) {
        this.drivetrainPose = drivetrainPoseSupplier;
        this.drivetrainVelocityX = drivetrainVelocityXSupplier;
        this.drivetrainVelocityY = drivetrainVelocityYSupplier;
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
        return 1.0;
    }

    /**
     * Math for modifying driver joystick input to robot
     * 
     * @param controllerInput the controller input for drivetrain x/y movement
     * @return the velocity output to the drivetrain
     */
    public double driveJoystickMath(double controllerInput) {
        // invert controllerInput (because the default controller direction is stupid)
        return MathUtil.applyDeadband(-controllerInput, OperatorConstants.driveDeadband) * this.driveSpeedMultiplier() * OperatorConstants.maxSpeed;
    }

    /**
     * The drivetrain rotation amount
     * 
     * @param controllerInput the controller input for drivetrain rotation
     * @return the rotation rate for drivetrain rotation
     */
    public double drivetrainRotationAmount(double controllerInput) {
        // invert controllerInput (because the default controller direction is stupid)
        return MathUtil.applyDeadband(-controllerInput, OperatorConstants.driveDeadband) * OperatorConstants.maxAngularRate;
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
     * The target angle for the shooter hood, in radians. The target angle is between 5*pi/36 radians (25 degrees) and 5*pi/18 radians (50 degrees).
     * 
     * @return The target hood angle, in radians
     */
    public double getHoodAngleTarget() {
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
