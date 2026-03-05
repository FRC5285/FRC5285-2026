package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.BucketOutConstants;

public class BucketOutSubsystem extends SubsystemBase {

    private final TalonFX rollerMotor; 
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
    
    public BucketOutSubsystem() {
        rollerMotor = new TalonFX(); // add id

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        rollerMotor.getConfigurator().apply(config);
    }


    public void start() {
        rollerMotor.setControl(dutyCycle.withOutput());
    }

   
    public void stop() {
        rollerMotor.setControl(dutyCycle.withOutput(0.0));
    }


    /** Returns a command that runs start() once */
    public Command startCommand() {
        return runOnce(this::start);
    }

    /** Returns a command that runs stop() once */
    public Command stopCommand() {
        return runOnce(this::stop);
    }

    @Override
    public void periodic() {
        
    }
}