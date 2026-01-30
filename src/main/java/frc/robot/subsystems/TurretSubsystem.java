package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;


import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class TurretSubsystem extends SubsystemBase {
    // Instance variables go here
    
    private final TalonFX turretMotor = new TalonFX(TurretConstants.motorCanId); 
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    public double turretTargetPosition = 0;
    Encoder m_encoder = new Encoder(TurretConstants.channel_a, TurretConstants.channel_b);

    public TurretSubsystem() {

        TalonFXConfiguration configs = new TalonFXConfiguration();
        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = TurretConstants.CruiseVelocity;   
        mm.MotionMagicAcceleration = TurretConstants.ACceleration;    
        mm.MotionMagicJerk = TurretConstants.Jerk;
        configs.MotionMagic = mm;

        Slot0Configs slot0 =configs.Slot0;
        slot0.kS = TurretConstants.kS;
        slot0.kV = TurretConstants.kV;
        slot0.kA = TurretConstants.kA;
        slot0.kP = TurretConstants.kp;
        slot0.kI = TurretConstants.ki;
        slot0.kD = TurretConstants.kd;
        configs.Slot0 = slot0;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretMotor.setPosition(0);
        turretMotor.getConfigurator().apply(configs);

        m_encoder.reset();
        m_encoder.setDistancePerPulse(0.5 / TurretConstants.m_steps); //m_steps should be 2048 but im lazy
        

        SendableRegistry.add(this, "Turret");
        SmartDashboard.putData(this);
    }

    // Other methods go here

    public Command stopTurret() {
        return runOnce(() -> turretMotor.stopMotor());
    }

    @Override
    public void periodic() {
        double currentPos = turretMotor.getPosition().getValueAsDouble();
        double encoderPos = m_encoder.getDistance();
        turretTargetPosition = -encoderPos; //negative since spin wrong direction
        turretMotor.setControl(motionMagicRequest.withPosition(turretTargetPosition).withSlot(0));
        // check if motor reached the target within tolerance
        SmartDashboard.putNumber("rotations", currentPos);
        SmartDashboard.putNumber("traget", turretTargetPosition);
        SmartDashboard.putNumber("error", (currentPos-turretTargetPosition));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        
    }
}
