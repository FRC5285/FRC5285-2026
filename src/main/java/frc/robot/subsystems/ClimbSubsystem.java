package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ClimbConstants;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.DutyCycleEncoder;


/*
 * CALIBRATE:
 * PIDs and profiler for both climbMotor and rotateMotor
 * goals for PIDs
 * lidar offset
 * motor tolerances
 * 
 * 
 * 2/19/2026
 * look into the tolerance for both PIDs because they're different from each other
 * enableContinuousOutput method
 */

public class ClimbSubsystem extends SubsystemBase {

  private TalonFX climbMotor;
  private TalonFX rotateMotor;

  private ProfiledPIDController climbPID;
  private ProfiledPIDController rotatePID;


  DutyCycleEncoder rotateEncoder = new DutyCycleEncoder(ClimbConstants.encoderChannel, 1.0, ClimbConstants.encoderStartRotations);

  private Rev2mDistanceSensor lidarSensor = new Rev2mDistanceSensor(Port.kMXP, Unit.kMillimeters, RangeProfile.kHighSpeed);

  public ClimbSubsystem() {

    climbMotor = new TalonFX(ClimbConstants.climbMotorID);

    climbPID = new ProfiledPIDController(ClimbConstants.ckP, ClimbConstants.ckI, ClimbConstants.ckD, new TrapezoidProfile.Constraints(ClimbConstants.cmaxA, ClimbConstants.cmaxV));
    climbPID.setGoal(ClimbConstants.maxExtension);
    climbPID.setTolerance(0.025);

    rotateMotor = new TalonFX(ClimbConstants.rotateMotorID);

    rotatePID = new ProfiledPIDController(ClimbConstants.rkP, ClimbConstants.rkI, ClimbConstants.rkD, new TrapezoidProfile.Constraints(ClimbConstants.rmaxA, ClimbConstants.rmaxV));
    rotatePID.setGoal(ClimbConstants.rotateInitialRotations);
    rotatePID.setTolerance(0.075);
    // rotatePID.enableContinuousInput(0.0, 1.0);

    lidarSensor.setAutomaticMode(true);

    SendableRegistry.add(this, "Climb");
    SmartDashboard.putData(this);
  }

  //ground -> ladder
  public Command Climb() {
    return runOnce(() -> {
      rotatePID.setGoal(ClimbConstants.rotateGoalRotations); //latch robot (rotateMotor) onto the ladder
    })
    .andThen(new WaitUntilCommand(() -> rotatePID.atGoal())) //wait until rotatePID is at its goal
    .andThen(
      runOnce(() -> {
        climbPID.setGoal(ClimbConstants.minExtension); //raise robot (climbMotor) up onto the ladder
      })
    ); 
  }

  // climb during auton
  public Command AutonClimb() {
    return runOnce(() -> {
      rotatePID.setGoal(ClimbConstants.rotateGoalRotations); //latch robot (rotateMotor) onto the ladder
    })
    .andThen(new WaitUntilCommand(() -> rotatePID.atGoal())) //wait until rotatePID is at its goal
    .andThen(
      runOnce(() -> {
        climbPID.setGoal(ClimbConstants.middleExtension); //raise robot (climbMotor) up onto the ladder
      })
    ); 
  }

  //ladder -> ground
  public Command Unclimb() {

    return runOnce(() -> {
      climbPID.setGoal(ClimbConstants.maxExtension); //lower robot (climbMotor) onto the ground
    })
    .andThen(new WaitUntilCommand(() -> climbPID.atGoal())) //wait until climbPID is at its goal
    .andThen(
      runOnce(() -> {
        rotatePID.setGoal(ClimbConstants.rotateInitialRotations); //then unlatch robot (rotateMotor) from ladder
      })
    ); 
  }

  //reset the "I" value for the motor PID 
  public void resetPID() {
    rotatePID.reset(rotateEncoder.get());

    double lidarPosition = getLidarMeters();
    climbPID.reset(lidarPosition);
  }

  //gets the lidar distance
  public double getLidarMeters() {

    return lidarSensor.getRange() / 1000.0 - ClimbConstants.lidarOffset;
  }

  // add new method "public boolean isUnclimbed()" - returns if goal positions are at unclimbed positions and pids are in tolerance
  // this method is needed for checking if the robot should be able to move at the start of teleop
  // otherwise the robot will start moving while it's climbed which is not good

  public boolean isUnclimbed() {
    return this.climbPID.atGoal()
    && this.rotatePID.atGoal()
    && this.climbPID.getGoal().position == ClimbConstants.maxExtension
    && this.rotatePID.getGoal().position == ClimbConstants.rotateInitialRotations;
  }

  //might have to invert 
  @Override
  public void periodic() {
    double lidarPosition = getLidarMeters();
    double climbNewMotorSpeed = climbPID.calculate(lidarPosition);
    climbMotor.setVoltage(climbNewMotorSpeed);

    double motorPosition = rotateEncoder.get();
    double rotateNewMotorSpeed = rotatePID.calculate(motorPosition);
    rotateMotor.setVoltage(rotateNewMotorSpeed);
  }

  @Override 
  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("Lidar Distance", () -> getLidarMeters(), null);
    builder.addDoubleProperty("Climb PID Goal", () -> this.climbPID.getGoal().position, null);
    builder.addDoubleProperty("Rotate PID Goal", () ->  this.rotatePID.getGoal().position, null);
    builder.addDoubleProperty("Rotate Motor Rotations", () -> this.rotateEncoder.get(), null);
  }
}