package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ClimbSubsystem extends SubsystemBase {

      //private TalonFX climbMotor1;
    //private TalonFX climbMotor2;
    //private ProfiledPIDController climbPID = new ProfiledPIDController(0, 0, 0, null);
    //Encoder climbEncoder = new Encoder(0,1);
                
    //climbMotor1 = new TalonFX(ClimbConstants.motorID);
    //climbMotor2 = new TalonFX(ClimbConstants.motorID);
    //climbMotor1.setPosition(0,0);
    //climbMotor2.setPosition(0,0);
    //climbEncoder.setDistancePerPulse(0.04/256/16);

  public ClimbSubsystem() {
    
  }

  @Override
  public void periodic() {

  }

    @Override 
    public void initSendable(SendableBuilder builder){
      
      //builder.addDoubleProperty("Goal Rotations", () -> m_encoder.getDistance(), null);
      //builder.addDoubleProperty("Motor Rotations", () -> climbMotor1.getPosition().getValueAsDouble(), null);
      //builder.addDoubleProperty("Motor Rotations", () -> climbMotor2.getPosition().getValueAsDouble(), null);
    }
  }