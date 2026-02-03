package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShiftUtil;
import frc.robot.Constants.LEDConstants;

//global usage, only one pattern at a time, 
//theLED.currentPattern = trans_flag; //first index is speed in Hz


public class LedSubsystem extends SubsystemBase {
    // Instance variables go here
    
    public LedSubsystem() {


        SendableRegistry.add(this, "LEDs");
        SmartDashboard.putData(this);
    }

    // Other methods go here
    public void setPattern(double[] pattern) {
        ledPatternState = null;
        currentPattern = pattern;
    }
    
    public Command stopMotor() {
        return runOnce(() -> LED.stopMotor());
    }

    @Override
    public void periodic() {
        if (currentPattern != null) {
            ledPattern(currentPattern);
            LED.set(currentValue);
    }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Can score", ()->ShiftUtil.canScore(), null);
    }
}
