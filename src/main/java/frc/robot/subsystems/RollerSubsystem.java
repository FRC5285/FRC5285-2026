package frc.robot.subsystems;
import frc.robot.Constants.RollerConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {

    private final TalonFX rollerMotor; 
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
    
    public RollerSubsystem() {
        rollerMotor = new TalonFX(RollerConstants.ROLLER_MOTOR_ID);

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

    /** Returns a command that runs stop() once */
    public Command stopCommand() {
        return runOnce(this::stop);
    }
}