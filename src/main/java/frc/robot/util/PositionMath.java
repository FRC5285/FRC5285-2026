package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LookupTableConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;

public class PositionMath {

    private final LookupTable speedTable;
    private final LookupTable timeOfFlightTable;

    private Supplier<Pose2d> drivetrainPose;
    private Supplier<Double> drivetrainVelocityX;
    private Supplier<Double> drivetrainVelocityY;
    private Supplier<Double> drivetrainVelocityRotation;
    private Supplier<Double> turretRotation;

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

        // Set previous drivetrain rotation target
        this.lastRotation = this.drivetrainPose.get().getRotation();

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
     */
    public void setSuppliers(
        Supplier<Pose2d> drivetrainPoseSupplier,
        Supplier<Double> drivetrainVelocityXSupplier,
        Supplier<Double> drivetrainVelocityYSupplier,
        Supplier<Double> drivetrainVelocityRotationSupplier,
        Supplier<Double> turretRotationSupplier
    ) {
        // Set suppliers
        this.drivetrainPose = drivetrainPoseSupplier;
        this.drivetrainVelocityX = drivetrainVelocityXSupplier;
        this.drivetrainVelocityY = drivetrainVelocityYSupplier;
        this.drivetrainVelocityRotation = drivetrainVelocityRotationSupplier;
        this.turretRotation = turretRotationSupplier;
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
        // Save compute
        if (!this.lastCalcPose.equals(this.drivetrainPose.get())) {
            this.calculateShootVector();
        }

        double dist = this.shootVector.getNorm();

        return this.speedTable.getOutput(dist);
    }

    /**
     * If shooting is a good idea
     * 
     * @return if the error for the shoot on the move calculation is greater than acceptable
     */
    public boolean shouldShoot() {
        return this.timeOfFlightTable.getCalcError() > LookupTableConstants.acceptableError;
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
        Transform2d turretPoseDiff = this.getTurretPose().minus(this.drivetrainPose.get());
        return this.drivetrainVelocityX.get() - turretPoseDiff.getY() * this.drivetrainVelocityRotation.get();
    }

    /** The turret Y velocity, Field-Centric */
    public double getTurretYVelocity() {
        Transform2d turretPoseDiff = this.getTurretPose().minus(this.drivetrainPose.get());
        return this.drivetrainVelocityY.get() + turretPoseDiff.getX() * this.drivetrainVelocityRotation.get();
    }

    /** The field-centric turret velocity as a vector represented by a Translation2d */
    public Translation2d getTurretVelocityVector() {
        Transform2d turretPoseDiff = this.getTurretPose().minus(this.drivetrainPose.get());
        double rotationVelocity = this.drivetrainVelocityRotation.get();
        return new Translation2d(
            this.drivetrainVelocityX.get() - turretPoseDiff.getY() * rotationVelocity,
            this.drivetrainVelocityY.get() + turretPoseDiff.getX() * rotationVelocity
        );
    }

    /**
     * Whether the robot is in its alliance zone
     * 
     * @return Whether the robot is in its alliance zone
     */
    public boolean inAllianceZone() {
        Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (currentAlliance == Alliance.Blue && this.drivetrainPose.get().getX() < FieldConstants.blueHubCenterX) {
            return true;
        }
        if (currentAlliance == Alliance.Red && this.drivetrainPose.get().getX() > FieldConstants.redHubCenterX) {
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
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return FieldConstants.blueHubCenterX;
        }
        return FieldConstants.redHubCenterX;
    }

    /** Resets the robot alliance according to driver station data. Runs whenever robot is enabled. */
    public void resetSide() {
        this.hubPose = new Pose2d(this.getAllianceLineX(), FieldConstants.midLineY, new Rotation2d());
    }

    /**
     * The target rotation for the turret
     * 
     * @return The turret rotation target, in radians
     */
    public double getTurretRotationTarget() {
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
            dRH = new Translation2d(this.getAllianceLineX() - this.lastCalcPose.getX(), this.lastCalcPose.getY());
        }

        this.shootVector = this.timeOfFlightTable.sotmCalc(this.getTurretVelocityVector(), dRH);

        return this.shootVector;
    }
}
