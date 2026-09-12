package frc.robot.subsystems;

import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.units.measure.Angle;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.tunable.Tunables;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.rotation.DutyCycleEncoder;
import frc.robot.Constants.TurretConstants;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.util.PositionMath;

import static org.wpilib.units.Units.Rotations;

import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;


public class TurretSubsystem extends SubsystemBase implements ComplexTunable{
    // Instance variables go here

    private final PositionMath positionMath;
    private final TalonFX turretMotor = new TalonFX(TurretConstants.motorCanId, CANBus.systemcore(TurretConstants.motorCanBus));
    private final TalonFX shooterMotor = new TalonFX(TurretConstants.ShooterMotorCanId, CANBus.systemcore(TurretConstants.ShooterMotorCanBus)); 
    private final TalonFX shooterMotor2 = new TalonFX(TurretConstants.ShooterMotor2CanId, CANBus.systemcore(TurretConstants.ShooterMotor2CanBus));

    private final MotionMagicVelocityVoltage motionMagicRequestShoooter = new MotionMagicVelocityVoltage(0);

    private final ProfiledPIDController turretPID = new ProfiledPIDController(TurretConstants.kp, TurretConstants.ki, TurretConstants.kd, new TrapezoidProfile.Constraints(TurretConstants.turretMaxV, TurretConstants.turretMaxA));
    private final SimpleMotorFeedforward turretFeedforward = new SimpleMotorFeedforward(TurretConstants.kS, TurretConstants.kV);

    private double turretTargetPosition = 0;
    private double shooterTargetRPS = 0;

    private double easyCRT;

    private DutyCycleEncoder encoderA = new DutyCycleEncoder(TurretConstants.channel_a);
    private DutyCycleEncoder encoderB = new DutyCycleEncoder(TurretConstants.channel_b);

    private Supplier<Angle> enc1 = () -> { return Rotations.of(encoderA.get()); };
    private Supplier<Angle> enc2 = () -> { return Rotations.of(encoderB.get()); };

    private boolean isDefending = false;

    private EasyCRTConfig easyCrt =
        new EasyCRTConfig(enc1, enc2)
            .withCommonDriveGear(
             /* commonRatio (mech:drive) */ 1.0,
                /* driveGearTeeth */ 200,
                /* encoder1Pinion */ 19,
                /* encoder2Pinion */ 21)
            .withAbsoluteEncoderOffsets(Rotations.of(TurretConstants.encoderAOffset), Rotations.of(TurretConstants.encoderBOffset)) // set after mechanical zero
            .withMechanismRange(Rotations.of(TurretConstants.min_range), Rotations.of(TurretConstants.max_range))
            .withMatchTolerance(Rotations.of(TurretConstants.match_tolerance))
            .withAbsoluteEncoderInversions(false, false)
    ;

    private EasyCRT easyCrtSolver = new EasyCRT(easyCrt);

    public TurretSubsystem(PositionMath positionMath) {

        this.positionMath = positionMath;

        TalonFXConfiguration ShooterConfigs = new TalonFXConfiguration();   

        turretMotor.setPosition(0);
        this.turretPID.setGoal(0.0);

        MotionMagicConfigs mm_s = new MotionMagicConfigs();
        mm_s.MotionMagicAcceleration = TurretConstants.S_ACceleration;
        mm_s.MotionMagicJerk = TurretConstants.S_Jerk;

        ShooterConfigs.MotionMagic = mm_s;

        Slot1Configs slot1 = ShooterConfigs.Slot1;
        slot1.kS = TurretConstants.S_kS;
        slot1.kV = TurretConstants.S_kV;
        slot1.kA = TurretConstants.S_kA;
        slot1.kP = TurretConstants.S_kp;
        slot1.kI = TurretConstants.S_ki;
        slot1.kD = TurretConstants.S_kd;
        ShooterConfigs.Slot1 = slot1;

        ShooterConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        shooterMotor.getConfigurator().apply(ShooterConfigs);

        shooterMotor.setPosition(0);

        shooterMotor2.setPosition(0);
        shooterMotor2.getConfigurator().apply(ShooterConfigs);

        Tunables.publish("Turret/Tunables", this);

        turretAngle();
    }

    /**
     * The current turret angle, in rotations
     * 
     * @return the current turret angle, in rotations
     */
    public double turretAngle() {
        easyCrtSolver.getAngleOptional().ifPresent(mechAngle -> {
            this.easyCRT = mechAngle.in(Rotations);
        });
        return this.easyCRT;
    }

    public void resetPIDs() {
        this.turretAngle();
        this.turretPID.reset(this.easyCRT);
    }

    public Command defendBegin() {
        return runOnce(() -> {
            this.isDefending = true;
        });
    }

    public Command defendEnd() {
        return runOnce(() -> {
            this.isDefending = false;
        });
    }

    @Override
    public void periodic() {
        turretAngle();

        this.shooterTargetRPS = this.positionMath.getFlywheelSpeedTarget();
        this.shooterTargetRPS = Math.min(TurretConstants.shooterMaxSpeed, Math.max(TurretConstants.shooterMinSpeed, this.shooterTargetRPS));

        this.turretTargetPosition = this.positionMath.getTurretRotationTarget() / (2.0 * Math.PI);
        this.turretTargetPosition = Math.min(TurretConstants.turretPIDMax, Math.max(TurretConstants.turretPIDMin, this.turretTargetPosition));

        if (!this.easyCrtSolver.getLastStatus().name().equals("OK") || this.isDefending) {
            this.turretTargetPosition = this.easyCRT;
            this.resetPIDs();
            turretMotor.setVoltage(0.0);
        } else {
            double turretPIDCalc = this.turretPID.calculate(this.easyCRT, turretTargetPosition);
            double turretFFCalc = this.turretFeedforward.calculate(this.turretPID.getSetpoint().velocity);
            turretMotor.setVoltage(-(turretPIDCalc + turretFFCalc));
        }

        shooterMotor.setControl(motionMagicRequestShoooter.withVelocity(-shooterTargetRPS).withSlot(1));
        shooterMotor2.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));        

        Telemetry.log("Turret", this);
    }

    @Override
    public void logTo(TelemetryTable table) {
        table.log("Easycrt output", this.easyCRT);
        table.log("Turret target", this.turretTargetPosition);
        table.log("Encoder A", (this.encoderA.get()));
        table.log("Encoder B", (this.encoderB.get()));
        table.log("Turret error", Math.abs(this.turretTargetPosition - this.easyCRT));
        table.log("turret debug", this.easyCrtSolver.getLastStatus().name());

        table.log("Shooter target RPS", this.shooterTargetRPS);   
        table.log("Shooter current RPS", this.shooterMotor.getVelocity().getValueAsDouble()); 
        table.log("Shooter error", Math.abs(this.shooterTargetRPS + this.shooterMotor.getVelocity().getValueAsDouble()));
    }

    @Override
    public void publishTunable(TunableTable table) {
        table.publishDouble("kS", () -> this.turretFeedforward.getKs(), (newKs) -> {this.turretFeedforward.setKs(newKs); this.resetPIDs();});
        table.publishDouble("kV", () -> this.turretFeedforward.getKv(), (newKv) -> {this.turretFeedforward.setKv(newKv); this.resetPIDs();});
        table.publish("Turret PID", this.turretPID);
    }
}