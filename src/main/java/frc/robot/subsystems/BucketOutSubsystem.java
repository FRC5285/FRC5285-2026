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
        rollerMotor = new TalonFX(BucketOutConstants.MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        rollerMotor.getConfigurator().apply(config);
    }


    public void spinForward() {
        rollerMotor.setControl(dutyCycle.withOutput(BucketOutConstants.SPEEDForwards));
    }

    public void spinReverse() {
        rollerMotor.setControl(dutyCycle.withOutput(-BucketOutConstants.SPEEDBackwards));
    }

    public void stop() {
        rollerMotor.setControl(dutyCycle.withOutput(0.0));
    }


    public Command forwardCommand(double seconds) {
        return run(this::spinForward)
                .withTimeout(seconds)
                .andThen(stopCommand());
    }

    
    public Command reverseCommand(double seconds) {
        return run(this::spinReverse)
                .withTimeout(seconds)
                .andThen(stopCommand());
    }

    
    public Command stopCommand() {
        return runOnce(this::stop);
    }


    public Command startCommand() {
        return forwardCommand(BucketOutConstants.forwardSeconds)
        .andThen(reverseCommand(BucketOutConstants.backwardSeconds))
        .repeatedly();
    }
}