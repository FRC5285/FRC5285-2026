package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

import frc.robot.Constants.BucketOutConstants;
import frc.robot.util.PositionMath;

public class BucketOutSubsystem extends SubsystemBase {

    private final PositionMath positionMath;

    private final TalonFX rollerMotor; 
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private boolean isOn = false;
    private boolean doReverse = false;
    private boolean lastCycleReversed = false;
    private double startTime;
    private double lastForwardTime = 0.0;
    
    public BucketOutSubsystem(PositionMath positionMath) {
        this.positionMath = positionMath;

        rollerMotor = new TalonFX(BucketOutConstants.MOTOR_ID, BucketOutConstants.CANBUS_ID);

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
        return runOnce(() -> {
            this.isOn = true;
            this.doReverse = false;
            this.startTime = Timer.getTimestamp();
            this.lastCycleReversed = false;
        });
    }

    @Override
    public void periodic() {
        if (this.isOn && this.positionMath.shouldShoot()) {
            double currentTime = Timer.getTimestamp();
            if ( // less than 0.5 seconds since motor last told to go forward (no more than 0.5 second reverse)
                currentTime - this.lastForwardTime < 0.5
                // AND
                && (
                    ( // more than 1 second since motor start, and slow motor
                        currentTime - this.startTime > 1.0
                        && Math.abs(this.rollerMotor.getVelocity().getValueAsDouble()) < 3.0
                    ) // OR the last control was to reverse the motor - motor always reverses for set amount of time
                    || this.lastCycleReversed
                )
            ) {
                this.spinReverse();
                this.lastCycleReversed = true;
            } else {
                if (this.lastCycleReversed == true) {
                    this.lastCycleReversed = false;
                    this.startTime = currentTime;
                }
                this.spinForward();
                this.lastForwardTime = currentTime;
            }
        } else if (this.doReverse) {
            this.spinReverse();
        } else {
            rollerMotor.setControl(dutyCycle.withOutput(0.0));
        }
    }
}