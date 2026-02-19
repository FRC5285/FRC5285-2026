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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.PositionMath;

public class RobotContainer implements Sendable {

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /** Handles most of the math for the robot. Do not create new instances in a subsystem, instead import this specific object in the constructor. */
    private final PositionMath positionMath = new PositionMath();

    private final TurretSubsystem turret = new TurretSubsystem();

    private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrain::addVisionMeasurement, () -> this.drivetrain.getPose(), positionMath);

    /** The driver controller */
    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.driverControllerPort);

    /** Drive with controller joystick rotation */
    private final SwerveRequest.FieldCentric driveFree = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    /** Drive with automatic rotation */
    private final SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withHeadingPID(OperatorConstants.rotationP, OperatorConstants.rotationI, OperatorConstants.rotationD);

    public RobotContainer() {
        // Set suppliers for math
        this.positionMath.setSuppliers(() -> this.drivetrain.getPose(), () -> this.drivetrain.getVelocityX(), () -> this.drivetrain.getVelocityY(), () -> this.drivetrain.getVelocityRotation(), () -> this.turret.turretAngle());

        // Configure controller bindings
        this.configureDrivetrainBinding();
        this.configureBindings();

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
        new Trigger(() -> !this.positionMath.bumpTurn() && this.driverController.rightBumper().getAsBoolean()).whileTrue(
            this.drivetrain.applyRequest(() ->
                this.driveFree.withVelocityX(this.positionMath.driveJoystickMath(driverController.getLeftY(), driverController.getLeftTriggerAxis()))
                    .withVelocityY(this.positionMath.driveJoystickMath(driverController.getLeftX(), driverController.getLeftTriggerAxis()))
                    .withRotationalRate(this.positionMath.driveRotationMath(driverController.getRightX(), driverController.getLeftTriggerAxis()))
            )
        );

        // Use auto rotation
        this.driverController.rightBumper().onFalse(
            this.drivetrain.getDefaultCommand()
        );
    }

    /** Configure controller bindings */
    private void configureBindings() {

    }

    /** Resets the field side and pose the robot is on. Runs when robot is enabled and auton was not used. */
    public void resetSide() {
        this.positionMath.resetSide();

        this.drivetrain.resetSide();
        this.drivetrain.resetPose(this.positionMath.drivetrainStartPosition());

        this.visionSubsystem.resetSimPose(this.drivetrain.getPose());
    }

    /** Resets the PIDs. Runs when robot is enabled. */
    public void resetPIDs() {
        this.drive.HeadingController.reset();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
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
