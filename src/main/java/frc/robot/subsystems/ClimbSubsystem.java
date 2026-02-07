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
    
    climbPID.setTolerance(ClimbConstants.lidarDistanceTolerance);
    lidarSensor.setAutomaticMode(true);
  }

  /*****************************************************************************
   (get lidar distance value -> fine tune pid to achieve that distance/height)
  *****************************************************************************/

  /* 
  public Command fineTunePID(Pose2d goHere, DrivetrainAligningTo whatAligningTo, double distFromObject, boolean doNormalPid, double lidarTime, double lidarTolerance) {
        return runOnce(() -> {
            this.xPID.reset(this.getState().Pose.getX());
            this.yPID.reset(this.getState().Pose.getY());
            this.rPID.reset(this.getState().Pose.getRotation().getRadians());
            this.rPID.enableContinuousInput(0.0, 2 * Math.PI);
            this.xPID.setGoal(goHere.getX());
            this.yPID.setGoal(goHere.getY());
            this.rPID.setGoal(goHere.getRotation().getRadians());
            this.thingAligningTo = whatAligningTo;
        })
        .andThen(
            run(() -> {
                this.setControl(
                    drivePID.withVelocityX(this.xPID.calculate(this.getState().Pose.getX()) * invertMult)
                    .withVelocityY(this.yPID.calculate(this.getState().Pose.getY()) * invertMult)
                    .withRotationalRate(this.rPID.calculate(this.getState().Pose.getRotation().getRadians()))
                );
            })
            .until(() -> this.xPID.atGoal() && this.yPID.atGoal() && this.rPID.atGoal())
            .withTimeout(AutoConstants.fineTuneMaxTime)
            .onlyIf(() -> doNormalPid)
        )
        .andThen(
            runOnce(() -> {
                this.lidarPID.reset(this.getLidarMeters());
                this.lidarPID.setTolerance(lidarTolerance);
                this.lidarPID.setGoal(distFromObject + RobotConstantsMeters.lidarBumperDistance);
                this.lidarGoal = distFromObject + RobotConstantsMeters.lidarBumperDistance;
                this.lidarOn = true;
            })
        )
        .andThen(
            run(() -> {
                this.setControl(
                    lidarDrive.withVelocityX(-this.lidarPID.calculate(this.getLidarMeters()))
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
                );
            })
            .onlyIf(() -> distFromObject >= 0.0 && this.getLidarMeters() >= RobotConstantsMeters.lidarBumperDistance)
            .until(() -> this.getLidarMeters() < RobotConstantsMeters.lidarBumperDistance || this.lidarPID.atGoal())
            .withTimeout(lidarTime)
        )
        .andThen(() -> {
            this.setControl(lidarDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            this.lidarOn = false;
        });
    }

    public Command moveBack() {
        return runOnce(() -> {
            this.setControl(
                lidarDrive.withVelocityX(AutoConstants.algaeMoveBackSpeedMPS).withVelocityY(0).withRotationalRate(0.0)
            );
        })
        .andThen(new WaitCommand(AutoConstants.algaeMoveBackSeconds))
        .andThen(
            runOnce(() -> {
                this.setControl(lidarDrive.withVelocityX(AutoConstants.algaeMoveBackSpeedMPS).withVelocityY(0).withRotationalRate(AutoConstants.algaeRotateSpeed));
            })
        )
        .andThen(new WaitCommand(AutoConstants.algaeRotateSeconds))
        .andThen(
            runOnce(() -> {
                this.setControl(lidarDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            })
        );
    }
    */ 

  public double getLidarMeters() {
    return lidarSensor.getRange() / 1000.0 - ClimbConstants.lidarOffset;
  }

  @Override
  public void periodic() {

  }

    @Override 
    public void initSendable(SendableBuilder builder){
      builder.addDoubleProperty("Lidar Distance", () -> this.getLidarMeters(), null);
      builder.addDoubleProperty("Lidar Goal", () -> this.lidarGoal, null);
      builder.addBooleanProperty("Lidar On", () -> this.lidarOn, null);
      builder.addDoubleProperty("Motor Rotations", () -> climbMotor.getPosition().getValueAsDouble(), null);
    }
  }