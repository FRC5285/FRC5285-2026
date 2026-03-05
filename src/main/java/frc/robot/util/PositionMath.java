package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;

public class PositionMath {


    // Speed of robot at controller max
    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    // Rotation rate of robot at controller max (in radians per second)
    private final double maxAngularRate = 2 * Math.PI;

    public PositionMath() {
    
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
        return MathUtil.applyDeadband(-controllerInput, OperatorConstants.driveDeadband) * this.driveSpeedMultiplier() * this.maxSpeed;
    }

    /**
     * The drivetrain rotation amount
     * 
     * @param controllerInput the controller input for drivetrain rotation
     * @return the rotation rate for drivetrain rotation
     */
    public double drivetrainRotationAmount(double controllerInput) {
        // invert controllerInput (because the default controller direction is stupid)
        return MathUtil.applyDeadband(-controllerInput, OperatorConstants.driveDeadband) * this.maxAngularRate;
    }

    /**
     * The target speed for the turret flywheel
     * 
     * @return The target flywheel speed
     */
    public double getFlywheelSpeedTarget() {
        return 0.0;
    }

    /**
     * The target angle for the turret
     * 
     * @return The target turret angle
     */
    public double getTurretAngleTarget() {
        return 0.0;
    }

    /**
     * The target rotation for the turret
     * 
     * @return The turret rotation target
     */
    public double getTurretRotationTarget() {
        return 0.0;
    }
}
