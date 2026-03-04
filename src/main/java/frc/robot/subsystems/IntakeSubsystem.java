// atrocious ass code

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
//import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeID);
    private final MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0);

    private final TalonFX lower = new TalonFX(IntakeConstants.lowerID);
    // private final MotionMagicVoltage motionMagicRequest1 = new
    // MotionMagicVoltage(0);

    double intakeSpeed = IntakeConstants.intakeSpeed; // radians per sec
    double toleranceSpeed = 16.0; // PID tolerance
    boolean stopped = false;

    // Trapezoid profile
    final TrapezoidProfile m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(IntakeConstants.maxVel, IntakeConstants.maxAcc));

    // Final target of angle rot, 0 rps
    TrapezoidProfile.State m_goal = new TrapezoidProfile.State(0, 0);
    TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    // create a position closed-loop request, voltage output, slot 0 configs
    PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

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
        intakeMotor.setPosition(0);
        intakeMotor.getConfigurator().apply(configs);

        // in init function, set slot 0 gains
        var slot1Configs = new Slot1Configs();
        slot1Configs.kS = IntakeConstants.kS1;
        slot1Configs.kV = IntakeConstants.kV1;
        slot1Configs.kP = IntakeConstants.kP1;
        slot1Configs.kI = IntakeConstants.kI1;
        slot1Configs.kD = IntakeConstants.kD1;
        configs.Slot1 = slot1Configs;

        lower.setPosition(0);
        lower.getConfigurator().apply(configs);

        SendableRegistry.add(this, "Turret Intake");
        SmartDashboard.putData(this);
    }

    // Other methods go here
    public Command beginIntake() {
        return runOnce(() -> {
            stopped = false;
            intakeSpeed = IntakeConstants.intakeSpeed;
        });
    }

    public Command endIntake() {
        return runOnce(() -> {
            stopped = true;
            intakeSpeed = 0;
        });
    }

    public Command setPosition(double angle) {
        return runOnce(() -> {
            // Final target of angle rot, 0 rps
            m_goal = new TrapezoidProfile.State(angle * 2 * Math.PI, 0);
            m_setpoint = new TrapezoidProfile.State();

            // create a position closed-loop request, voltage output, slot 0 configs
            m_request = new PositionVoltage(0).withSlot(1);
        });
    }

    // Baguette
    @Override
    public void periodic() {
        if (!stopped) {
            intakeMotor.setControl(motionMagicRequest.withVelocity(intakeSpeed).withSlot(0));
        } else {
            intakeMotor.stopMotor();
        }

        // calculate the next profile setpoint
        m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);

        // send the request to the device
        m_request.Position = m_setpoint.position;
        m_request.Velocity = m_setpoint.velocity;
        lower.setControl(m_request);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Intake Motor Rotations per second",
                () -> this.intakeMotor.getVelocity().getValueAsDouble(), null);
        builder.addDoubleProperty("Intake Motor Radians Per Second",
                () -> this.intakeMotor.getVelocity().getValueAsDouble() * 2 * Math.PI, null);
        builder.addDoubleProperty("Lowering Motor Rotations", () -> this.lower.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Target Speed", () -> this.intakeSpeed, null);
    }
}
