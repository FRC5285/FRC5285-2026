package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

import java.util.function.Supplier;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.PositionMath;
import static edu.wpi.first.units.Units.Rotations;

import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;


public class TurretSubsystem extends SubsystemBase {
    // Instance variables go here

    private final PositionMath positionMath;
    
    private final TalonFX turretMotor = new TalonFX(TurretConstants.motorCanId); 
    private final TalonFX shooterMotor = new TalonFX(TurretConstants.ShooterMotorCanId); 
    private final TalonFX shooterMotor2 = new TalonFX(TurretConstants.ShooterMotor2CanId);

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage motionMagicRequestShoooter = new MotionMagicVelocityVoltage(0);
    
    public double turretTargetPosition = 0;
    public double shooterTargetRPS = 0;
    public double currentPos;
    Encoder encoder = new Encoder(TurretConstants.channel_a, TurretConstants.channel_b);
    Encoder encoder_1 = new Encoder(TurretConstants.channel_a_a, TurretConstants.channel_b_b);

////////////////////////////////////////////////
/// 
    Supplier<Angle> enc1 = () -> { return Rotations.of(encoder.get()); };
    Supplier<Angle> enc2 = () -> { return Rotations.of(encoder.get()); };

    EasyCRTConfig easyCrt =
        new EasyCRTConfig(enc1, enc2)
            .withCommonDriveGear(
                /* commonRatio (mech:drive) */ 20.0,
                /* driveGearTeeth */ 200,
                /* encoder1Pinion */ 19,
                /* encoder2Pinion */ 21)
            .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0)) // set after mechanical zero
            .withMechanismRange(Rotations.of(-2.0), Rotations.of(3.0)) // -360 deg to +720 deg
            .withMatchTolerance(Rotations.of(0.06)) // ~1.08 deg at encoder2 for the example ratio
            .withAbsoluteEncoderInversions(false, false)
            .withCrtGearRecommendationConstraints(
                /* coverageMargin */ 1.2,
                /* minTeeth */ 15,
                /* maxTeeth */ 45,
                /* maxIterations */ 30);

    EasyCRT easyCrtSolver = new EasyCRT(easyCrt);

    public TurretSubsystem(PositionMath positionMath) {

        this.positionMath = positionMath;

        TalonFXConfiguration configs = new TalonFXConfiguration();
        TalonFXConfiguration ShooterConfigs = new TalonFXConfiguration();   
        TalonFXConfiguration Shooter2Configs = new TalonFXConfiguration();     

        FeedbackConfigs fdb_shooter = ShooterConfigs.Feedback;
        fdb_shooter.SensorToMechanismRatio = TurretConstants.shooter_ratio; //shooter motor, 1:1
        FeedbackConfigs fdb = configs.Feedback;
        fdb.SensorToMechanismRatio = TurretConstants.gear_ratio_on_drive_ring; //from motor to gear box to ring


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
        mm_s.MotionMagicAcceleration = TurretConstants.S_ACceleration;
        mm_s.MotionMagicJerk = TurretConstants.S_Jerk;

        Slot1Configs slot1 = ShooterConfigs.Slot1;
        slot1.kS = TurretConstants.S_kS;
        slot1.kV = TurretConstants.S_kV;
        slot1.kA = TurretConstants.S_kA;
        slot1.kP = TurretConstants.S_kp;
        slot1.kI = TurretConstants.S_ki;
        slot1.kD = TurretConstants.S_kd;
        ShooterConfigs.Slot1 = slot1;

        ShooterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotor.setPosition(0);

        shooterMotor.getConfigurator().apply(ShooterConfigs);

        
//////////////////////////////////////////////////////////////
/// second shooter motor in follower mode
        shooterMotor2.setPosition(0);
        shooterMotor2.getConfigurator().apply(ShooterConfigs);

        shooterMotor2.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));

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

    /**
     * The current turret angle, in rotations
     * 
     * @return the current turret angle, in rotations
     */
    public double turretAngle() {
        easyCrtSolver.getAngleOptional().ifPresent(mechAngle -> {
            double easyCRTrotations = mechAngle.in(Rotations);
            turretMotor.setPosition(easyCRTrotations);
        });
        return turretMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        turretAngle();
        
        turretTargetPosition = positionMath.getTurretRotationTarget()/TurretConstants.convert_to_rotations_from_radians; 
        shooterTargetRPS = positionMath.getFlywheelSpeedTarget();
        shooterTargetRPS = Math.min(shooterTargetRPS, 100);

        currentPos = turretMotor.getPosition().getValueAsDouble(); //very shitty 

        turretMotor.setControl(motionMagicRequest.withPosition(turretTargetPosition));
        shooterMotor.setControl(motionMagicRequestShoooter.withVelocity(shooterTargetRPS));
        // check if motor reached the target within tolerance
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        SmartDashboard.putNumber("turret base target", turretTargetPosition);
        SmartDashboard.putNumber("radians per second", shooterMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("turret base error", (currentPos-turretTargetPosition)); 
    }
}