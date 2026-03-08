package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.BucketOutConstants;

public class BucketOutSubsystem extends SubsystemBase {

    private final TalonFX rollerMotor; 
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private boolean isOn = false;
    private boolean doReverse = false;
    
    public BucketOutSubsystem() {
        rollerMotor = new TalonFX(BucketOutConstants.MOTOR_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        rollerMotor.getConfigurator().apply(config);
    }


    private void spinForward() {
        rollerMotor.setControl(dutyCycle.withOutput(BucketOutConstants.SPEEDForwards));
    }

    private void spinReverse() {
        rollerMotor.setControl(dutyCycle.withOutput(BucketOutConstants.SPEEDBackwards));
    }

    private void stop() {
        this.isOn = false;
        this.doReverse = false;
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

    public Command setReverse() {
        return runOnce(() -> {this.doReverse = true; this.isOn = false;});
    }

    
    public Command stopCommand() {
        return runOnce(this::stop);
    }


    public Command startCommand() {
        return runOnce(() -> {this.isOn = true; this.doReverse = false;});
    }

    @Override
    public void periodic() {
        if (this.isOn) {
            double currentTime = Timer.getFPGATimestamp() % (BucketOutConstants.forwardSeconds + BucketOutConstants.backwardSeconds);
            if (currentTime < BucketOutConstants.forwardSeconds) {
                this.spinForward();
            } else {
                this.spinReverse();
            }
        } else if (this.doReverse) {
            this.spinReverse();
        } else {
            rollerMotor.setControl(dutyCycle.withOutput(0.0));
        }
    }
}