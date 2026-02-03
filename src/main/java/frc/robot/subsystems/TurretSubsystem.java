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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.PositionMath;


public class TurretSubsystem extends SubsystemBase {
    // Instance variables go here
    
    private final TalonFX turretMotor = new TalonFX(TurretConstants.motorCanId); 
    private final TalonFX shooterMotor = new TalonFX(TurretConstants.ShooterMotorCanId); 

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage motionMagicRequestShoooter = new MotionMagicVelocityVoltage(0);

    public double turretTargetPosition = 0;
    public double shooterTargetRPS = 0;
    Encoder encoder = new Encoder(TurretConstants.channel_a, TurretConstants.channel_b);
    Encoder encoder_1 = new Encoder(TurretConstants.channel_a_a, TurretConstants.channel_b_b);

    public TurretSubsystem() {

        TalonFXConfiguration configs = new TalonFXConfiguration();
        TalonFXConfiguration ShooterConfigs = new TalonFXConfiguration();

        FeedbackConfigs fdb_shooter = ShooterConfigs.Feedback;
        fdb_shooter.SensorToMechanismRatio = 1.0; //shooter motor, 1:1
        FeedbackConfigs fdb = configs.Feedback;
        fdb.SensorToMechanismRatio = 90.0; //from motor to gear box to ring


        MotionMagicConfigs mm = new MotionMagicConfigs();
        mm.MotionMagicCruiseVelocity = TurretConstants.CruiseVelocity;   
        mm.MotionMagicAcceleration = TurretConstants.ACceleration;    
        mm.MotionMagicJerk = TurretConstants.Jerk;
        configs.MotionMagic = mm;

        Slot0Configs slot0 = configs.Slot0;
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
/////////////////////////////////////////////////////////////
        MotionMagicConfigs mm_s = new MotionMagicConfigs();
        mm_s.MotionMagicAcceleration = 320.0;
        mm_s.MotionMagicJerk = 3200;

        Slot1Configs slot1 = ShooterConfigs.Slot1;
        slot0.kS = TurretConstants.kS;
        slot0.kV = TurretConstants.kV;
        slot0.kA = TurretConstants.kA;
        slot0.kP = TurretConstants.kp;
        slot0.kI = TurretConstants.ki;
        slot0.kD = TurretConstants.kd;
        ShooterConfigs.Slot1 = slot1;

        ShooterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shooterMotor.setPosition(0);
        shooterMotor.getConfigurator().apply(ShooterConfigs);

        encoder.reset();
        encoder_1.reset();

        encoder.setDistancePerPulse(0.5 / TurretConstants.m_steps); //m_steps should be 2048 but im lazy
        encoder_1.setDistancePerPulse(0.5 / TurretConstants.m_steps); //m_steps should be 2048 but im lazy

        SendableRegistry.add(this, "Turret");
        SmartDashboard.putData(this);


    }

    // Other methods go here

    public Command stopTurret() {
        return runOnce(() -> turretMotor.stopMotor());
    }

    @Override
    public void periodic() {
        
        PositionMath positionMath = new PositionMath(null, null, null);
        turretTargetPosition = positionMath.getTurretRotationTarget();

        shooterTargetRPS = positionMath.getFlywheelSpeedTarget();

        double currentPos = 0; //rmotionMagicRequesteplace with supplier from position math

        turretMotor.setControl(motionMagicRequest.withPosition(turretTargetPosition * TurretConstants.gear_ratio_on_drive_ring).withSlot(0));
        shooterMotor.setControl(motionMagicRequestShoooter.withVelocity(shooterTargetRPS));
        // check if motor reached the target within tolerance
        SmartDashboard.putNumber("angles", currentPos);
        SmartDashboard.putNumber("traget", turretTargetPosition);
        SmartDashboard.putNumber("error", (currentPos-turretTargetPosition));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        //rotation angle, 
    }
}
