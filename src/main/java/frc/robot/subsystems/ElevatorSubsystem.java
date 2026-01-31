package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private final TalonFX elevatorMotor;
  private final TalonFX followerMotor;
  private final ElevatorState elevatorState;
  private final Encoder elevatorEncoder;
  private final ProfiledPIDController elevatorPID;
  private final ElevatorFeedforward elevatorFeedforward;
  private boolean motorOverride = false;

  public elevatorLastSelectedHeight goingToHeight = elevatorLastSelectedHeight.THREE;
    //private TalonFX climbMotor1;
    //private TalonFX climbMotor2;
    //private ProfiledPIDController climbPID = new ProfiledPIDController(0, 0, 0, null);
    //Encoder climbEncoder = new Encoder(0,1);
                
    //climbMotor1 = new TalonFX(ClimbConstants.motorID);
    //climbMotor2 = new TalonFX(ClimbConstants.motorID);
    //climbMotor1.setPosition(0,0);
    //climbMotor2.setPosition(0,0);
    //climbEncoder.setDistancePerPulse(0.04/256/16);

  public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(ElevatorConstants.elevatorMotorID);
    followerMotor = new TalonFX(ElevatorConstants.followMotorID);
    // topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitchID);
    // bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitchID);
    elevatorEncoder = new Encoder(ElevatorConstants.encoderA, ElevatorConstants.encoderB);
    elevatorEncoder.setDistancePerPulse(ElevatorConstants.encoderPulseDist);
    elevatorPID = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
      new TrapezoidProfile.Constraints(ElevatorConstants.maxV, ElevatorConstants.maxA)
    );
    elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    elevatorPID.setTolerance(ElevatorConstants.goalRange);
    elevatorPID.setGoal(0);
    
    followerMotor.setControl(new Follower(elevatorMotor.getDeviceID(), false));

    elevatorState = new ElevatorState();
  }

  public Command goToPosition(DoubleSupplier getTargetPosition) {
    return runOnce(() -> {
      motorOverride = false;
      elevatorPID.setGoal(getTargetPosition.getAsDouble() - ElevatorConstants.encoderOffset);
    });
  }

  public Command goToPosition(elevatorLastSelectedHeight goToHeight) {
    final double targetPos;
    if (goToHeight == elevatorLastSelectedHeight.ONE) {
      targetPos = ElevatorConstants.rung1Position;
    } else if (goToHeight == elevatorLastSelectedHeight.TWO) {
      targetPos = ElevatorConstants.rung2Position;
    } else {
      targetPos = ElevatorConstants.rung3Position;
    } 
    return this.goToPosition(() -> targetPos);
  }

  @Override
  public void periodic() {
    double pidCalc = elevatorPID.calculate(elevatorEncoder.getDistance());
    double ffwdCalc = elevatorFeedforward.calculate(elevatorPID.getSetpoint().velocity);
    if (motorOverride == false) this.elevatorMotor.setVoltage(pidCalc + ffwdCalc);
  }

  public boolean reachedGoal() {
    return elevatorPID.atGoal();
  }

  public Command goToLevel1Position(){
    return runOnce(() -> this.goingToHeight = elevatorLastSelectedHeight.ONE).andThen(goToPosition(()-> elevatorState.getRung1Position()));
  }

  public Command goToLevel2Position(){
    return runOnce(() -> this.goingToHeight = elevatorLastSelectedHeight.TWO).andThen(goToPosition(()-> elevatorState.getRung2Position()));
  }

  public Command goToLevel3Position(){
    return runOnce(() -> this.goingToHeight = elevatorLastSelectedHeight.THREE).andThen(goToPosition(()-> elevatorState.getRung3Position()));
  }

  public Command setToLevel1Position() {
    return runOnce(() -> this.goingToHeight = elevatorLastSelectedHeight.ONE);
  }

  public Command setToLevel2Position() {
    return runOnce(() -> this.goingToHeight = elevatorLastSelectedHeight.TWO);
  }

  public Command setToLevel3Position() {
    return runOnce(() -> this.goingToHeight = elevatorLastSelectedHeight.THREE);
  }

  public Command goToBottomPosition(){
    return goToPosition(()-> 0.0);
  }

  public Command elevatorUp() {
    return runOnce(() -> elevatorPID.setGoal(Math.min(elevatorPID.getGoal().position + 0.05, ElevatorConstants.maxHeight)));
  }

  public Command elevatorDown() {
    return runOnce(() -> elevatorPID.setGoal(Math.max(elevatorPID.getGoal().position - 0.05, 0.0)));
  }

  public Command stopElevator() {
    return runOnce(() -> {
      motorOverride = true;
      elevatorMotor.stopMotor();
    });
  }

  public enum elevatorLastSelectedHeight{
    ONE,
    TWO,
    THREE,
  }

  public class ElevatorState implements Sendable{

    private double rung1Position = ElevatorConstants.rung1Position;
    private double rung2Position = ElevatorConstants.rung2Position;
    private double rung3Position = ElevatorConstants.rung3Position;
    private double maxHeight = ElevatorConstants.maxHeight;

    public double getRung1Position() {
      return rung1Position;
    }

    public void setRung1Position(double rung1Position) {
      this.rung1Position = rung1Position;
    }

    public double getRung2Position() {
      return rung2Position;
    }

    public void setRung2Position(double rung2Position) {
      this.rung2Position = rung2Position;
    }

    public double getRung3Position() {
      return rung3Position;
    }

    public void setRung3Position(double rung3Position) {
      this.rung3Position = rung3Position;
    }

    public double getMaxHeight() {
      return maxHeight;
    }

    public void setMaxHeight(double maxHeight) {
      this.maxHeight = maxHeight;
    }

    public ElevatorState(){
      SendableRegistry.add(this, "Elevator State");
      SmartDashboard.putData(this);
    }
    
    @Override 
    public void initSendable(SendableBuilder builder){
      builder.setSmartDashboardType("RobotPreferences");

      builder.addDoubleProperty("Goal Height", () -> elevatorPID.getSetpoint().position, (newVal) -> {});
      builder.addDoubleProperty("Current Height", () -> elevatorEncoder.getDistance(), (newVal) -> {});
      builder.addDoubleProperty("Rung 1 Position", this::getRung1Position, this::setRung1Position);
      builder.addDoubleProperty("Rung 2 Position", this::getRung2Position, this::setRung2Position);
      builder.addDoubleProperty("Rung 3 Position", this::getRung3Position, this::setRung3Position);
      builder.addDoubleProperty("Max Height Position", this::getMaxHeight, this::setMaxHeight);
      builder.addBooleanProperty("R1", () -> goingToHeight == elevatorLastSelectedHeight.ONE, null);
      builder.addBooleanProperty("R2", () -> goingToHeight == elevatorLastSelectedHeight.TWO, null);
      builder.addBooleanProperty("R3", () -> goingToHeight == elevatorLastSelectedHeight.THREE, null);

      //builder.addDoubleProperty("Goal Rotations", () -> m_encoder.getDistance(), null);
      //builder.addDoubleProperty("Motor Rotations", () -> climbMotor1.getPosition().getValueAsDouble(), null);
        //builder.addDoubleProperty("Motor Rotations", () -> climbMotor2.getPosition().getValueAsDouble(), null);
    }
  }
}