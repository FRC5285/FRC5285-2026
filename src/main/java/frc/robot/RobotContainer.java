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
import frc.robot.subsystems.TurretIntakeSubsystem;
import frc.robot.util.PositionMath;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

    //private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final PositionMath positionMath = new PositionMath();

    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.driverControllerPort);

    //private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final TurretSubsystem turret = new TurretSubsystem(positionMath);

    private final TurretIntakeSubsystem turretIntake = new TurretIntakeSubsystem();

    public RobotContainer() {
        //this.configureDrivetrainBinding();
        this.configureBindings();
        this.turretIntake.setNewSpeed(120);
    }

    /** Configures the drivetrain drive command */
    private void configureDrivetrainBinding() {
        
    }

    /** Configure controller bindings */
    private void configureBindings() {
        this.driverController.b().onTrue(this.turretIntake.beginIntake());
        this.driverController.b().onFalse(this.turretIntake.endIntake());
        this.driverController.x().onTrue(this.turretIntake.reverseIntake());
        this.driverController.x().onFalse(this.turretIntake.endIntake());
    }

    /** Resets the field side and pose the robot is on */
    public void resetSide() {
     
    }

    /** Resets the PIDs */
    public void resetPIDs() {

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
