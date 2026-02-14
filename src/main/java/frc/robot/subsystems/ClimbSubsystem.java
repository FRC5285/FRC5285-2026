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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimbConstants;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

/*
 * a lidar will be used to determine the distance (height) to the rung
 * then with the distance, use that value to determine the motor PID (climbPID) for that height
 * 
 * one motor & one lidar will be used (no encoder)
 * 
 * i need two distances for when robot is fully climbed (on the ladder) and not climbed (on the ground)
 * set pid to climb value
 * motor will go to that climb value
 * will use the lidar value
 */

public class ClimbSubsystem extends SubsystemBase {

  private TalonFX climbMotor;
  private ProfiledPIDController climbPID;

  private Rev2mDistanceSensor lidarSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kHighSpeed);
  private double lidarGoal = 0.0;
  private boolean lidarOn = false;

  public ClimbSubsystem() {

    climbMotor = new TalonFX(ClimbConstants.motorID);
    climbMotor.setPosition(0,0);

    climbPID = new ProfiledPIDController(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD, new TrapezoidProfile.Constraints(ClimbConstants.maxA, ClimbConstants.maxV));
    climbPID.setGoal(ClimbConstants.maxExtension);
    climbPID.setTolerance(ClimbConstants.lidarDistanceTolerance);

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

  //**DONE** -- reset the "I" value for the motor PID 
  public Command resetPID() {

    double lidarPosition = getLidarMeters();
    climbPID.reset(lidarPosition);
  }

  //set PID to certain value
  /*public Command setPID() {

  }*/

  //**DONE** -- gets the lidar distance
  public double getLidarMeters() {

    return lidarSensor.getRange() / 1000.0 - ClimbConstants.lidarOffset;
  }

  //**DONE (for now)** -- might have to invert 
  @Override
  public void periodic() {

    climbPID.setGoal(getLidarMeters());

    double lidarPosition = getLidarMeters();
    double newMotorSpeed = climbPID.calculate(lidarPosition);
    climbMotor.setVoltage(newMotorSpeed);
  }

  @Override 
  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("Lidar Distance", () -> getLidarMeters(), null);
    builder.addDoubleProperty("PID Goal", () -> this.climbPID.getGoal().position, null);
    builder.addDoubleProperty("Motor Rotations", () -> climbMotor.getPosition().getValueAsDouble(), null);
  }
}