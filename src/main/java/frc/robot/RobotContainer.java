// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.PositionMath;

public class RobotContainer {

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final PositionMath positionMath = new PositionMath(() -> this.drivetrain.getState().Pose, () -> this.drivetrain.getVelocityX(), () -> this.drivetrain.getVelocityY());

    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.driverControllerPort);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public RobotContainer() {
        this.configureDrivetrainBinding();
        this.configureBindings();
    }

    /** Configures the drivetrain drive command */
    private void configureDrivetrainBinding() {
        this.drivetrain.setDefaultCommand(
            this.drivetrain.applyRequest(() ->
                this.drive.withVelocityX(this.positionMath.driveJoystickMath(driverController.getLeftY()))
                    .withVelocityY(this.positionMath.driveJoystickMath(driverController.getLeftX()))
                    .withRotationalRate(this.positionMath.drivetrainRotationAmount(driverController.getRightX()))
            )
        );
    }

    /** Configure controller bindings */
    private void configureBindings() {

    }

    /** Resets the field side and pose the robot is on */
    public void resetSide() {
        this.drivetrain.resetSide();
        this.drivetrain.resetPose(this.positionMath.drivetrainStartPosition());
    }

    /** Resets the PIDs */
    public void resetPIDs() {

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
