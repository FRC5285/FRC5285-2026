package frc.robot.subsystems;
import frc.robot.Constants.RollerConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {

    private final TalonFX rollerMotor; 
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
    
    public RollerSubsystem() {
        rollerMotor = new TalonFX(RollerConstants.ROLLER_MOTOR_ID, RollerConstants.ROLLER_MOTOR_BUS);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        rollerMotor.getConfigurator().apply(config);
    }


    private void start() {
        rollerMotor.setControl(dutyCycle.withOutput(RollerConstants.speed));
    }

    private void startFast() {
        rollerMotor.setControl(dutyCycle.withOutput(RollerConstants.fastSpeed));
    }

    private void reverse() {
        rollerMotor.setControl(dutyCycle.withOutput(-RollerConstants.fastSpeed));
    }

    private void stop() {
        rollerMotor.setControl(dutyCycle.withOutput(0.0));
    }


    /** Returns a command that runs start() once */
    public Command startCommand() {
        return runOnce(this::start);
    }

    public Command startFastCommand() {
        return runOnce(this::startFast);
    }

    public Command reverseCommand() {
        return runOnce(this::reverse);
    }

    /** Returns a command that runs stop() once */
    public Command stopCommand() {
        return runOnce(this::stop);
    }
}