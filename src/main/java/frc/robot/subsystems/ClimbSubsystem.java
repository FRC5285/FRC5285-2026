package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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
import frc.robot.Constants.ClimbConstants;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

/*
 * 2/18/26
 * adjust the climb and unclimb commands to sequential commands to account for the rotating motor 
 * (may have an encoder for rotateMotor)
 * 
 * climb command:
 * rotateMotor first to latch onto the ladder, wait until rotatePID is within tolerance, then climbMotor to raise the robot up
 * 
 * unclimb command:
 * climbMotor to lower the robot down, wait until climbPID is within tolerance, then rotateMotor to unlatch from the ladder
 * 
 */

public class ClimbSubsystem extends SubsystemBase {

  private TalonFX climbMotor;
  private TalonFX rotateMotor;
  private ProfiledPIDController climbPID;
  private ProfiledPIDController rotatePID;

  private double goalRotations;

  private Rev2mDistanceSensor lidarSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kHighSpeed);
  private double lidarGoal = 0.0;
  private boolean lidarOn = false;

  public ClimbSubsystem() {

    climbMotor = new TalonFX(ClimbConstants.climbMotorID);
    climbMotor.setPosition(0,0);

    climbPID = new ProfiledPIDController(ClimbConstants.ckP, ClimbConstants.ckI, ClimbConstants.ckD, new TrapezoidProfile.Constraints(ClimbConstants.cmaxA, ClimbConstants.cmaxV));
    climbPID.setGoal(ClimbConstants.maxExtension);

    rotateMotor = new TalonFX(ClimbConstants.rotateMotorID);
    rotateMotor.setPosition(0,0);

    goalRotations = 0.0;

    rotatePID = new ProfiledPIDController(ClimbConstants.rkP, ClimbConstants.rkI, ClimbConstants.rkD, new TrapezoidProfile.Constraints(ClimbConstants.rmaxA, ClimbConstants.rmaxV));
    rotatePID.setGoal(goalRotations);

    lidarSensor.setAutomaticMode(true);
  }

  //starting at the ground
  public Command Climb() {

    return runOnce(() -> {
      climbPID.setGoal(ClimbConstants.minExtension); 
    });
  }

  //on the ladder
  public Command Unclimb() {

    return runOnce(() -> {
      climbPID.setGoal(ClimbConstants.maxExtension);
    });
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