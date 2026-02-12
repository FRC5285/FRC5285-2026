package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(IntakeConstants.motorCanId);
    private final MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0);
    private double targetPosition = 0;

    public IntakeSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        var talonFXConfigs = new TalonFXConfiguration();

        // configure Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.cruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.acceleration;
        motionMagicConfigs.MotionMagicJerk = IntakeConstants.jerk;
        configs.MotionMagic = motionMagicConfigs;

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = IntakeConstants.kS;
        slot0Configs.kV = IntakeConstants.kV;
        slot0Configs.kA = IntakeConstants.kA;
        slot0Configs.kP = IntakeConstants.kP;
        slot0Configs.kI = IntakeConstants.kI;
        slot0Configs.kD = IntakeConstants.kD;
        configs.Slot0 = slot0Configs;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motor.setPosition(0);
        motor.getConfigurator().apply(configs);

        SendableRegistry.add(this, "Intake");
        SmartDashboard.putData(this);
    }

    // Other methods go here


    @Override
    public void periodic() {
        motor.setControl(motionMagicRequest.withVelocity(160).withSlot(0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        
    }
}
