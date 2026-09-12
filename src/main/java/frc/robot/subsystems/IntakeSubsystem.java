package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;

import org.wpilib.hardware.rotation.DutyCycleEncoder;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.tunable.Tunables;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase implements ComplexTunable {
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeID, CANBus.systemcore(IntakeConstants.intakeBus));

    private final TalonFX lower = new TalonFX(IntakeConstants.lowerID, CANBus.systemcore(IntakeConstants.lowerBus));
    private final TalonFX lowerFollower = new TalonFX(IntakeConstants.followerId, CANBus.systemcore(IntakeConstants.followerBus));

    private final SimpleMotorFeedforward intakeFeedforward = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV);
    private final ProfiledPIDController lowerPID = new ProfiledPIDController(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD, new TrapezoidProfile.Constraints(IntakeConstants.maxVel, IntakeConstants.maxAcc));
// tune feedfoward and pid below later!!!!!1
    private final SimpleMotorFeedforward lowerFeedforward_2 = new SimpleMotorFeedforward(IntakeConstants.followerS, IntakeConstants.followerV); 
    private final ProfiledPIDController lowerPID_2 = new ProfiledPIDController(IntakeConstants.followerP, IntakeConstants.followerI, IntakeConstants.followerD, new TrapezoidProfile.Constraints(IntakeConstants.maxVel_2, IntakeConstants.maxAcc_2));
   
    DutyCycleEncoder encoder = new DutyCycleEncoder(IntakeConstants.encoderChannel);
    DutyCycleEncoder encoder2 = new DutyCycleEncoder(IntakeConstants.encoderChannel_2);
    private double encoderAddedRotations = 0.0;
    private double encoderTotalRotations = 0.0;
    private double encoderPreviousRotations;

    private double encoder2_AddedRotations = 0.0;
    private double encoder2_TotalRotatoins = 0.0;
    private double encoder2_PreviousRotations = 0.0;

    public IntakeSubsystem() {
        this.lowerPID.setGoal(IntakeConstants.intakeRaisedValue);
        this.lowerPID_2.setGoal(IntakeConstants.intakeFollower_RaisedValue);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeMotor.getConfigurator().apply(configs);

        lower.setPosition(0.0);
        lowerFollower.setPosition(0.0);

        // lowerFollower.setControl(new Follower(intakeMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        this.encoderPreviousRotations = this.getEncoderPosition();
        this.encoder2_PreviousRotations = encoder2.get();

        Tunables.publish("Ground Intake/Tunables", this);
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
            this.lowerPID_2.setGoal(IntakeConstants.intakeFolllower_LoweredValue);
        });
    }

    public Command raiseIntake() {
        return runOnce(() -> {
            this.lowerPID.setGoal(IntakeConstants.intakeSecondRaisedValue);
            this.lowerPID_2.setGoal(IntakeConstants.intakeFollower_SecondRaisedValue);
        });
    }

    public Command followerUp() {
        return runOnce(() -> {
            this.lowerFollower.setVoltage(-2.0);
        });
    }

    public Command followerDown() {
        return runOnce(() -> {
            this.lowerFollower.setVoltage(2.0);
        });
    }

    public Command followerStop() {
        return runOnce(() -> {
            this.lowerFollower.setVoltage(0.0);
        });
    }

    private double getEncoderPosition() {
        return encoder.get();
    }
    

    public double getExtensionRotations() {
        return this.encoderTotalRotations;
    }

    public double getExtensionRotations_2() {
        return this.encoder2_TotalRotatoins;
    }

    public void resetPIDs() {
        this.lowerPID.reset(this.getExtensionRotations());
        this.lowerPID_2.reset(this.getExtensionRotations());
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
/* -------------------------------------------------------------------------- */
        double encoder2_Pos = encoder2.get();
        if (encoder2_Pos < 0.1 && this.encoder2_PreviousRotations > 0.9) {
            this.encoder2_AddedRotations += 1.0;
        }
        else if (encoder2_Pos > 0.9 && this.encoder2_PreviousRotations < 0.1) {
            this.encoder2_AddedRotations -= 1.0;
        }
        this.encoder2_PreviousRotations = encoder2_Pos;
        this.encoder2_TotalRotatoins = this.encoder2_AddedRotations + encoder2_Pos;
        /* -------------------------------------------------------------------------- */

        double pidCalc = this.lowerPID.calculate(this.getExtensionRotations());
        double ffCalc = this.intakeFeedforward.calculate(this.getExtensionRotations(), this.lowerPID.getSetpoint().velocity);

/* -------------------------------------------------------------------------- */

        double pidCalc_2 = this.lowerPID_2.calculate(this.encoder2_TotalRotatoins);
        double ffCalc_2 = this.lowerFeedforward_2.calculate(this.encoder2_TotalRotatoins, this.lowerPID_2.getSetpoint().velocity);
        /* -------------------------------------------------------------------------- */

        this.lower.setVoltage(-(pidCalc + ffCalc));

        /* -------------------------------------------------------------------------- */

        this.lowerFollower.setVoltage(-(pidCalc_2 + ffCalc_2));

        Telemetry.log("Ground Intake", this);
    }

    @Override
    public void logTo(TelemetryTable table) {
        table.log("Extension Value", this.getExtensionRotations());
        table.log("Extension goal", this.lowerPID.getGoal().position);

        table.log("extension value 2", this.getExtensionRotations_2());
        table.log("second extension goal", this.lowerPID_2.getGoal().position);

        table.log("lower PID", this.lowerPID);
        table.log("lower PID 2", this.lowerPID_2);
    }

    @Override
    public void publishTunable(TunableTable table) {
        table.publish("lower PID", this.lowerPID);
        table.publish("lower PID 2", this.lowerPID_2);

        table.publishDouble("kS", () -> this.intakeFeedforward.getKs(), (newKs) -> {this.intakeFeedforward.setKs(newKs); this.resetPIDs();});
        table.publishDouble("kV", () -> this.intakeFeedforward.getKv(), (newKv) -> {this.intakeFeedforward.setKv(newKv); this.resetPIDs();});

        table.publishDouble("kS follower", () -> this.lowerFeedforward_2.getKs(), (newKs) -> {this.lowerFeedforward_2.setKs(newKs); this.resetPIDs();});
        table.publishDouble("kV follower", () -> this.lowerFeedforward_2.getKv(), (newKv) -> {this.lowerFeedforward_2.setKv(newKv); this.resetPIDs();});
    }
}
 