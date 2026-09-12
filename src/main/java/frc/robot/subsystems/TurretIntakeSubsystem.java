package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.tunable.Tunables;

import frc.robot.Constants.TurretIntakeConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TurretIntakeSubsystem extends SubsystemBase implements ComplexTunable {
    private final TalonFX motor = new TalonFX(TurretIntakeConstants.motorCanId, CANBus.systemcore(TurretIntakeConstants.motorCanBus));
    private final MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0);

    double intakeSpeed = 0.0; // radians per sec, target speed

    public TurretIntakeSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        var talonFXConfigs = new TalonFXConfiguration();

        // configure Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = TurretIntakeConstants.cruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = TurretIntakeConstants.acceleration;
        motionMagicConfigs.MotionMagicJerk = TurretIntakeConstants.jerk;
        configs.MotionMagic = motionMagicConfigs;

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = TurretIntakeConstants.kS;
        slot0Configs.kV = TurretIntakeConstants.kV;
        slot0Configs.kA = TurretIntakeConstants.kA;
        slot0Configs.kP = TurretIntakeConstants.kP;
        slot0Configs.kI = TurretIntakeConstants.kI;
        slot0Configs.kD = TurretIntakeConstants.kD;
        configs.Slot0 = slot0Configs;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motor.setPosition(0);
        motor.getConfigurator().apply(configs);

        Tunables.publish("Turret Intake/Tunables", this);
    }

    // Other methods go here
    public boolean atTargetSpeed() {
        return Math.abs(this.motor.getVelocity().getValueAsDouble() - this.intakeSpeed) <= TurretIntakeConstants.speedTolerance;
    }

    public Command beginIntake() {
        return this.setSpeed(TurretIntakeConstants.intakeSpeed);
    }

    public Command reverseIntake() {
        return this.setSpeed(TurretIntakeConstants.reverseSpeed);
    }

    public Command endIntake() {
        return this.setSpeed(0.0);
    }

    private Command setSpeed(double speed) {
        return runOnce(() -> {
            this.setNewSpeed(speed);
        });
    }

    public void setNewSpeed(double speed) {
        this.intakeSpeed = speed;
        motor.setControl(motionMagicRequest.withVelocity(this.intakeSpeed).withSlot(0));
    }

    @Override
    public void periodic() {
        Telemetry.log("Turret Intake", this);
    }

    @Override
    public void logTo(TelemetryTable table) {
        table.log("Rotations per second", this.motor.getVelocity().getValueAsDouble());
        table.log("error", Math.abs(this.intakeSpeed - this.motor.getVelocity().getValueAsDouble()));
    }

    @Override
    public void publishTunable(TunableTable table) {
        table.publishDouble("Goal", () -> this.intakeSpeed, (newSpeed) -> this.setNewSpeed(newSpeed));
    }
}
