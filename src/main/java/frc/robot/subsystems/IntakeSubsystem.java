package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeID);

    private final TalonFX lower = new TalonFX(IntakeConstants.lowerID);

    private final ArmFeedforward intakeFeedforward = new ArmFeedforward(IntakeConstants.kS, IntakeConstants.kG, IntakeConstants.kV);
    private final ProfiledPIDController lowerPID = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, new TrapezoidProfile.Constraints(IntakeConstants.maxVel, IntakeConstants.maxAcc));

    private boolean canShutOffLower = false;
    private boolean doingIntake = false;
    DutyCycleEncoder encoder = new DutyCycleEncoder(IntakeConstants.encoderChannel, 2.0 * Math.PI, IntakeConstants.encoderStartValue);

    public IntakeSubsystem() {
        this.lowerPID.enableContinuousInput(0.0, 2.0 * Math.PI);
        this.lowerPID.setGoal(IntakeConstants.intakeRaisedValue);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeMotor.getConfigurator().apply(configs);

        SendableRegistry.add(this, "Ground Intake");
        SmartDashboard.putData(this);
    }

    // Other methods go here
    public Command beginIntake() {
        return runOnce(() -> {
            intakeMotor.setVoltage(IntakeConstants.intakeVolts);
            this.doingIntake = true;
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
            this.doingIntake = false;
        });
    }

    public Command lowerIntake() {
        return runOnce(() -> {
            this.lowerPID.setGoal(IntakeConstants.intakeLoweredValue);
            this.canShutOffLower = true;
        });
    }

    public Command raiseIntake() {
        return runOnce(() -> {
            this.lowerPID.setGoal(IntakeConstants.intakeRaisedValue);
            this.canShutOffLower = false;
        });
    }

    /** Gets the angle of the intake, in rotations. 0 is lowered angle */
    public double getAngleRadians() {
        return encoder.get();
    }

    public void resetPIDs() {
        this.lowerPID.reset(this.getAngleRadians());
    }

    @Override
    public void periodic() {
        double pidCalc = this.lowerPID.calculate(this.getAngleRadians());
        double ffCalc = this.intakeFeedforward.calculate(getAngleRadians(), this.lowerPID.getSetpoint().velocity);
        if (this.getAngleRadians() <= IntakeConstants.intakeLoweredEnoughValue && this.canShutOffLower && this.doingIntake) {
            this.lower.setVoltage(0.0); // 0.0
            this.resetPIDs();
        } else if (this.getAngleRadians() <= IntakeConstants.intakeLoweredEnoughValue && this.canShutOffLower && !this.doingIntake) {
            this.lower.setVoltage(0.0);
            this.resetPIDs();
        } else if (this.getAngleRadians() >= 1.15 && this.canShutOffLower) {
            this.lower.setVoltage(0.0); // 9.0
            this.resetPIDs();
        } else {
            // this.lower.setVoltage((pidCalc + ffCalc));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Intake Motor Rotations per second",
                () -> this.intakeMotor.getVelocity().getValueAsDouble(), null);
        builder.addDoubleProperty("Encoder Value", () -> this.getAngleRadians(), null);
        builder.addDoubleProperty("Lowering Motor Rotations", () -> this.lower.getPosition().getValueAsDouble(), null);
        // comment out after calibration
        builder.addDoubleProperty("kS", () -> this.intakeFeedforward.getKs(), (newKs) -> {this.intakeFeedforward.setKs(newKs); this.resetPIDs();});
        builder.addDoubleProperty("kG", () -> this.intakeFeedforward.getKg(), (newKg) -> {this.intakeFeedforward.setKg(newKg); this.resetPIDs();});
        builder.addDoubleProperty("kV", () -> this.intakeFeedforward.getKv(), (newKv) -> {this.intakeFeedforward.setKv(newKv); this.resetPIDs();});
        builder.addDoubleProperty("kP", () -> this.lowerPID.getP(), (newP) -> {this.lowerPID.setP(newP); this.resetPIDs();});
        builder.addDoubleProperty("kD", () -> this.lowerPID.getD(), (newD) -> {this.lowerPID.setD(newD); this.resetPIDs();});
    }
}
