package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ClimbConstants;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


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
 * 
 */

public class ClimbSubsystem extends SubsystemBase {

  private TalonFX climbMotor;
  private TalonFX rotateMotor;

  private ProfiledPIDController climbPID;
  private ProfiledPIDController rotatePID;

  private double goalRotations;

  private Rev2mDistanceSensor lidarSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kHighSpeed);

  public ClimbSubsystem() {

    climbMotor = new TalonFX(ClimbConstants.climbMotorID);
    climbMotor.setPosition(0,0);

    climbPID = new ProfiledPIDController(ClimbConstants.ckP, ClimbConstants.ckI, ClimbConstants.ckD, new TrapezoidProfile.Constraints(ClimbConstants.cmaxA, ClimbConstants.cmaxV));
    climbPID.setGoal(ClimbConstants.maxExtension);
    climbPID.setTolerance(5, 10);

    goalRotations = 0.0;

    rotateMotor = new TalonFX(ClimbConstants.rotateMotorID);
    rotateMotor.setPosition(0,0);

    rotatePID = new ProfiledPIDController(ClimbConstants.rkP, ClimbConstants.rkI, ClimbConstants.rkD, new TrapezoidProfile.Constraints(ClimbConstants.rmaxA, ClimbConstants.rmaxV));
    rotatePID.setGoal(goalRotations);
    rotatePID.setTolerance(5, 10);

    lidarSensor.setAutomaticMode(true);
  }

  //ground -> ladder
  public Command Climb() {

    return runOnce(() -> {
      goalRotations = ClimbConstants.rotateGoalRotations;
      rotatePID.setGoal(goalRotations); //latch robot (rotateMotor) onto the ladder
    })
    .andThen(new WaitUntilCommand(() -> rotatePID.atGoal())) //wait until rotatePID is at its goal
    .andThen(
      runOnce(() -> {
        climbPID.setGoal(ClimbConstants.minExtension);
      })
    ); //raise robot (climbMotor) up onto the ladder
  }

  //ladder -> ground
  public Command Unclimb() {

    return runOnce(() -> {
      climbPID.setGoal(ClimbConstants.maxExtension); //lower robot (climbMotor) onto the ground
      goalRotations = ClimbConstants.rotateGoalRotations * -1.0;
    })
    .andThen(new WaitUntilCommand(() -> climbPID.atGoal())) //wait until climbPID is at its goal
    .andThen(
      runOnce(() -> {
        rotatePID.setGoal(goalRotations);
      })
    ); //then unlatch robot (rotateMotor) from ladder
  }

  //reset the "I" value for the motor PID 
  public void resetPID() {

    double motorPosition = rotateMotor.getPosition().getValueAsDouble();
    rotatePID.reset(motorPosition);

    double lidarPosition = getLidarMeters();
    climbPID.reset(lidarPosition);
  }

  //gets the lidar distance
  public double getLidarMeters() {

    return lidarSensor.getRange() / 1000.0 - ClimbConstants.lidarOffset;
  }

  //might have to invert 
  @Override
  public void periodic() {

    climbPID.setGoal(getLidarMeters());

    double lidarPosition = getLidarMeters();
    double climbNewMotorSpeed = climbPID.calculate(lidarPosition);
    climbMotor.setVoltage(climbNewMotorSpeed);

    double motorPosition = rotateMotor.getPosition().getValueAsDouble();
    double rotateNewMotorSpeed = rotatePID.calculate(motorPosition);
    rotateMotor.set(rotateNewMotorSpeed);
  }

  @Override 
  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("Lidar Distance", () -> getLidarMeters(), null);
    builder.addDoubleProperty("Climb PID Goal", () -> this.climbPID.getGoal().position, null);
    builder.addDoubleProperty("Rotate PID Goal", () ->  this.rotatePID.getGoal().position, null);
    builder.addDoubleProperty("Motor Rotations", () -> climbMotor.getPosition().getValueAsDouble(), null);
  }
}