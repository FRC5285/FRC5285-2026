package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeID);

    private final TalonFX lower = new TalonFX(IntakeConstants.lowerID);
    private final TalonFX lowerFollower = new TalonFX(IntakeConstants.followerId);

    private final SimpleMotorFeedforward intakeFeedforward = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV);
    private final ProfiledPIDController lowerPID = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, new TrapezoidProfile.Constraints(IntakeConstants.maxVel, IntakeConstants.maxAcc));

    DutyCycleEncoder encoder = new DutyCycleEncoder(IntakeConstants.encoderChannel);
    private double encoderAddedRotations = 0.0;
    private double encoderTotalRotations = 0.0;
    private double encoderPreviousRotations;
    private double followerMultipler = IntakeConstants.followerMultiplier;

    public IntakeSubsystem() {
        this.lowerPID.setGoal(IntakeConstants.intakeRaisedValue);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeMotor.getConfigurator().apply(configs);

        lower.setPosition(0.0);
        lowerFollower.setPosition(0.0);

        // lowerFollower.setControl(new Follower(intakeMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        this.encoderPreviousRotations = this.getEncoderPosition();

        SendableRegistry.add(this, "Ground Intake");
        SmartDashboard.putData(this);
    }

    // Other methods go here
    public Command beginIntake() {
        return runOnce(() -> {
            intakeMotor.setVoltage(IntakeConstants.intakeVolts);
        });
    }

    public Command reverseIntake() {
        return runOnce(() -> {
            intakeMotor.setVoltage(-IntakeConstants.intakeVolts);
        });
    }

    public Command endIntake() {
        return runOnce(() -> {
            intakeMotor.stopMotor();
        });
    }

    public Command lowerIntake() {
        return runOnce(() -> {
            this.lowerPID.setGoal(IntakeConstants.intakeLoweredValue);
            this.followerMultipler = IntakeConstants.followerMultiplier;
        });
    }

    public Command raiseIntake() {
        return runOnce(() -> {
            this.lowerPID.setGoal(IntakeConstants.intakeSecondRaisedValue);
            this.followerMultipler = IntakeConstants.followerMultiplerUp;
        });
    }

    private double getEncoderPosition() {
        return encoder.get();
    }

    public double getExtensionRotations() {
        return this.encoderTotalRotations;
    }

    public void resetPIDs() {
        this.lowerPID.reset(this.getExtensionRotations());
    }

    @Override
    public void periodic() {
        double encoderPos = this.getEncoderPosition();
        if (encoderPos < 0.1 && this.encoderPreviousRotations > 0.9) {
            this.encoderAddedRotations += 1.0;
        } else if (encoderPos > 0.9 && this.encoderPreviousRotations < 0.1) {
            this.encoderAddedRotations -= 1.0;
        }
        this.encoderPreviousRotations = encoderPos;
        this.encoderTotalRotations = this.encoderAddedRotations + encoderPos;

        double pidCalc = this.lowerPID.calculate(this.getExtensionRotations());
        double ffCalc = this.intakeFeedforward.calculate(this.getExtensionRotations(), this.lowerPID.getSetpoint().velocity);
        this.lower.setVoltage(-(pidCalc + ffCalc));
        this.lowerFollower.setVoltage((pidCalc + ffCalc) * this.followerMultipler);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Intake Motor Rotations per second",
                () -> this.intakeMotor.getVelocity().getValueAsDouble(), null);
        builder.addDoubleProperty("Extension Value", () -> this.getExtensionRotations(), null);
        builder.addDoubleProperty("Extension goal", () -> this.lowerPID.getGoal().position, null);
        builder.addDoubleProperty("Lowering Motor Rotations", () -> this.lower.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("amps", () -> this.lower.getSupplyCurrent().getValueAsDouble(), null);
        // comment out after calibration
        builder.addDoubleProperty("kS", () -> this.intakeFeedforward.getKs(), (newKs) -> {this.intakeFeedforward.setKs(newKs); this.resetPIDs();});
        builder.addDoubleProperty("kV", () -> this.intakeFeedforward.getKv(), (newKv) -> {this.intakeFeedforward.setKv(newKv); this.resetPIDs();});
        builder.addDoubleProperty("kP", () -> this.lowerPID.getP(), (newP) -> {this.lowerPID.setP(newP); this.resetPIDs();});
        builder.addDoubleProperty("kD", () -> this.lowerPID.getD(), (newD) -> {this.lowerPID.setD(newD); this.resetPIDs();});
        builder.addDoubleProperty("follower multiplier", () -> this.followerMultipler, (newMult) -> this.followerMultipler = newMult);
    }
}
