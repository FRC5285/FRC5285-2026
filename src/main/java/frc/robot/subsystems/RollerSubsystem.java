package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {

    
    private final TalonFX rollerMotor;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    private static final int ROLLER_MOTOR_ID = 5;

    public RollerSubsystem() {
        rollerMotor = new TalonFX(ROLLER_MOTOR_ID);

        // Motor configuration
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        rollerMotor.getConfigurator().apply(config);
    }


    public void run() {
        rollerMotor.setControl(dutyCycle.withOutput(0.8));
    }

    
    public void stop() {
        rollerMotor.setControl(dutyCycle.withOutput(0.0));
    }
}


