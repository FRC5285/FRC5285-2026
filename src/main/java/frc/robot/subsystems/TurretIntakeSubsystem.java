package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private final TalonFX intakeMotor = new TalonFX(TurretIntakeConstants.intakeID);
    private final MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0);

    private final TalonFX lower = new TalonFX(TurretIntakeConstants.lowerID);
    private final MotionMagicVoltage motionMagicRequest1 = new MotionMagicVoltage(0);
    
    double intakeSpeed = 160; // radians per sec

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
        intakeMotor.setPosition(0);
        intakeMotor.getConfigurator().apply(configs);

        // in init function, set slot 0 gains
        var slot0Configs1 = new Slot0Configs();
        slot0Configs1.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs1.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs1.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs1.kI = 0; // no output for integrated error
        slot0Configs1.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        lower.getConfigurator().apply(slot0Configs1);

        SendableRegistry.add(this, "Turret Intake");
        SmartDashboard.putData(this);
    }

    // Other methods go here
    public Command beginIntake() {
        return run(() -> {
            intakeMotor.setControl(motionMagicRequest.withVelocity(intakeSpeed).withSlot(0));
        });
    }

    public Command endIntake() {
        return runOnce(() -> {
            intakeMotor.stopMotor();
        });
    }

    public Command setPosition() {
        return run(() -> {
            // Trapezoid profile with max velocity 80 rps, max accel 160 rps/s
            final TrapezoidProfile m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(80, 160)
            );
            // Final target of 200 rot, 0 rps
            TrapezoidProfile.State m_goal = new TrapezoidProfile.State(200, 0);
            TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

            // create a position closed-loop request, voltage output, slot 0 configs
            final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

            // calculate the next profile setpoint
            m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);

            // send the request to the device
            m_request.Position = m_setpoint.position;
            m_request.Velocity = m_setpoint.velocity;
            lower.setControl(m_request);
        });
    }

    @Override
    public void periodic() {
        beginIntake();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Rotations per second", () -> this.intakeMotor.getVelocity().getValueAsDouble(), null);
        builder.addDoubleProperty("Radians Per Second", () -> this.intakeMotor.getVelocity().getValueAsDouble() * 2 * 3.1415, null);
    }
}

