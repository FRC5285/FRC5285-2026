// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.TurretIntakeSubsystem;
import frc.robot.util.PositionMath;
import frc.robot.util.ShiftUtil;

public class RobotContainer implements Sendable {

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /** Handles most of the math for the robot. Do not create new instances in a subsystem, instead import this specific object in the constructor. */
    private final PositionMath positionMath = new PositionMath();

    private final TurretSubsystem turret = new TurretSubsystem(this.positionMath);

    private final IntakeSubsystem groundIntake = new IntakeSubsystem();

    private final TurretIntakeSubsystem turretIntake = new TurretIntakeSubsystem();

    private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain::addVisionMeasurement, () -> this.drivetrain.getPose(), positionMath);

    private final LedSubsystem ledSubsystem = new LedSubsystem();

    private final AutonSubsystem autonSubsystem = new AutonSubsystem(this.drivetrain, this.groundIntake, this.turretIntake, this.ledSubsystem, this.positionMath);

    /** The driver controller */
    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.driverControllerPort);

    /** The second controller */
    private final CommandXboxController secondController = new CommandXboxController(OperatorConstants.secondControllerPort);

    /** Drive with controller joystick rotation */
    private final SwerveRequest.FieldCentric driveFree = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /** Drive with automatic rotation */
    private final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withHeadingPID(OperatorConstants.rotationP, OperatorConstants.rotationI, OperatorConstants.rotationD);

    public RobotContainer() {
        // Set suppliers for math
        this.positionMath.setSuppliers(
            () -> this.drivetrain.getPose(),
            () -> this.drivetrain.getVelocityX(),
            () -> this.drivetrain.getVelocityY(),
            () -> this.drivetrain.getVelocityRotation(),
            () -> this.turret.turretAngle(),
            () -> this.autonSubsystem.isShooting(),
            () -> this.autonSubsystem.isClimbing()
        );

        // Configure controller bindings
        this.configureDrivetrainBinding();
        this.configureBindings();
        this.configureOtherTriggers();

        // Telemetry
        SendableRegistry.add(this, "RobotContainer");
        SmartDashboard.putData(this);
    }

    /** Configures the drivetrain drive commands */
    private void configureDrivetrainBinding() {
        // Default command (auto rotation)
        this.drivetrain.setDefaultCommand(
            this.drivetrain.applyRequest(() ->
                this.drive.withVelocityX(this.positionMath.driveJoystickMath(driverController.getLeftY(), driverController.getLeftTriggerAxis()))
                    .withVelocityY(this.positionMath.driveJoystickMath(driverController.getLeftX(), driverController.getLeftTriggerAxis()))
                    .withTargetDirection(this.positionMath.drivetrainRotationAmount())
            )
        );

        // Manual rotation when not over bump and with right bumper button down
        new Trigger(() -> !this.positionMath.bumpTurn() && this.driverController.leftBumper().getAsBoolean()).whileTrue(
            this.drivetrain.applyRequest(() ->
                this.driveFree.withVelocityX(this.positionMath.driveJoystickMath(driverController.getLeftY(), driverController.getLeftTriggerAxis()))
                    .withVelocityY(this.positionMath.driveJoystickMath(driverController.getLeftX(), driverController.getLeftTriggerAxis()))
                    .withRotationalRate(this.positionMath.driveRotationMath(driverController.getRightX(), driverController.getLeftTriggerAxis()))
            )
        );

        // Use auto rotation
        this.driverController.leftBumper().onFalse(
            this.drivetrain.getDefaultCommand()
        );
    }

    /** Configure controller bindings */
    private void configureBindings() {
        this.driverController.rightTrigger().onTrue(
            this.autonSubsystem.shootingOn()
        );

        this.driverController.rightTrigger().onFalse(
            this.autonSubsystem.shootingOff()
        );

        this.driverController.povLeft().onTrue(
            this.autonSubsystem.climbLeft()
            .andThen(this.drivetrain.applyRequest(() ->
                this.driveFree.withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
            ))
        );

        this.driverController.povRight().onTrue(
            this.autonSubsystem.climbRight()
            .andThen(this.drivetrain.applyRequest(() ->
                this.driveFree.withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
            ))
        );

        this.secondController.leftBumper().onTrue(
            this.autonSubsystem.intakeUp()
        );

        this.secondController.leftBumper().onTrue(
            this.autonSubsystem.intakeDown()
        );
    }

    /** Configure the other triggers */
    private void configureOtherTriggers() {
        new Trigger(() -> ShiftUtil.canScore() && this.ledSubsystem.getCurrentCommand() == null).onTrue(
            this.ledSubsystem.hubActive()
        );

        new Trigger(() -> !ShiftUtil.canScore() && this.ledSubsystem.getCurrentCommand() == null).onTrue(
            this.ledSubsystem.hubInactive()
        );

        new Trigger(() -> ShiftUtil.beforeShooting() && this.ledSubsystem.getCurrentCommand() == null).onTrue(
            this.ledSubsystem.preHub()
        );
    }

    /** Resets the field side and pose the robot is on. Runs when robot is enabled and auton was not used. */
    public void resetSide() {
        this.positionMath.resetSide();
        this.positionMath.resetLastRotation();

        this.drivetrain.resetSide();
        this.drivetrain.resetPose(this.positionMath.drivetrainStartPosition());

        ShiftUtil.resetShift();

        this.visionSubsystem.resetSimPose(this.drivetrain.getPose());
    }

    /** Resets the PIDs. Runs when robot is enabled. */
    public void resetPIDs() {
        this.positionMath.resetLastRotation();
        this.drive.HeadingController.reset();
        this.groundIntake.resetPIDs();
    }

    public Command getAutonomousCommand() {
        return this.autonSubsystem.autonCommand();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("PID Goal", () -> this.drive.HeadingController.getSetpoint(), null);
        builder.addDoubleProperty("Robot Heading", () -> this.drivetrain.getPose().getRotation().getRadians(), null);
        builder.addDoubleProperty("Goal Flywheel Speed", () -> this.positionMath.getFlywheelSpeedTarget(), null);
        builder.addDoubleProperty("Turret X Velocity", () -> this.positionMath.getTurretXVelocity(), null);
        builder.addDoubleProperty("Turret Y Velocity", () -> this.positionMath.getTurretYVelocity(), null);
    }
}
