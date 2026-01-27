package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.math.controller.PIDController;  
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
/*
 * ELEVATOR
 * - 1 or 2 motors
 * - if 2 motors, they will go same direction
 */

public class ClimbSubsystem extends SubsystemBase {
    // Instance variables go here

    //private TalonFX climbMotor1;
    //private TalonFX climbMotor2;
    //private ProfiledPIDController climbPID = new ProfiledPIDController(0, 0, 0, null);

    
    public ClimbSubsystem() {

        //climbMotor1 = new TalonFX(ClimbConstants.motorID);
        //climbMotor2 = new TalonFX(ClimbConstants.motorID);
        //climbMotor1.setPosition(0,0);
        //climbMotor2.setPosition(0,0);

        SendableRegistry.add(this, "Climber");
        SmartDashboard.putData(this);
    }

    // Other methods go here


    @Override
    public void periodic() {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        
    }
}
