package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
//import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretIntakeConstants;

// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TurretIntakeSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(TurretIntakeConstants.motorCanId);
    private final MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0);

    double intakeSpeed = TurretIntakeConstants.intakeSpeed; // radians per sec, target speed
    double tolerance = 1.0;
    boolean stopped = false;

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

        SendableRegistry.add(this, "Intake");
        SmartDashboard.putData(this);
    }

    // Other methods go here
    public boolean atTargetSpeed() {
        return (this.motor.getVelocity().getValueAsDouble() == intakeSpeed + tolerance || this.motor.getVelocity().getValueAsDouble() == intakeSpeed - tolerance);
    }

    public Command beginIntake() {
        return runOnce(() -> {
            stopped = false;
        });
    }

    public Command endIntake() {
        return runOnce(() -> {
            stopped = true;
            motor.stopMotor();
        });
    }

    @Override
    public void periodic() {
        if (!stopped) motor.setControl(motionMagicRequest.withVelocity(intakeSpeed).withSlot(0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Rotations per second", () -> this.motor.getVelocity().getValueAsDouble(), null);
        builder.addDoubleProperty("Radians Per Second", () -> this.motor.getVelocity().getValueAsDouble() * 2 * Math.PI, null);
        builder.addDoubleProperty("Goal", () -> this.intakeSpeed, null);
    }
}
