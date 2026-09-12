// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;

import org.wpilib.framework.TimedRobot;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /** If the robot has been in Auton before disable */
    private boolean usedAuton = false;
    /** If the robot has been in Teleop before disable */
    private boolean usedTeleop = false;

    public Robot() {
        this.m_robotContainer = new RobotContainer();

        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Telemetry.log("Robot", this.m_robotContainer);
    }

    @Override
    public void disabledInit() {
        // Resets booleans
        if (this.usedTeleop) {
            this.usedAuton = false;
            this.usedTeleop = false;
        }
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        this.m_robotContainer.resetPIDs();
        this.m_robotContainer.resetSide();
        this.usedAuton = true;
        this.m_autonomousCommand = this.m_robotContainer.getAutonomousCommand();

        if (this.m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(this.m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        this.usedTeleop = true;

        // CommandScheduler.getInstance().schedule(this.m_robotContainer.getTeleopStartCommand());

        // Resets robot field orientation only if Auton was not used
        if (!this.usedAuton) {
            this.m_robotContainer.resetSide();
        }

        this.m_robotContainer.resetPIDs();

        if (this.m_autonomousCommand != null) {
            this.m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void utilityInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void utilityPeriodic() {}

    @Override
    public void utilityExit() {}
}
