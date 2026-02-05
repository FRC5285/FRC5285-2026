package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ShiftUtil;
import frc.robot.Constants.LEDConstants;

//global usage, only one pattern at a time, 
//theLED.currentPattern = trans_flag; //first index is speed in Hz


public class LedSubsystem extends SubsystemBase {
    private final Spark LED = new Spark(LEDConstants.led_pin);
    public long[] ledPatternState = null;
    public double[] currentPattern = null;
    private double currentValue = 0.0;

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
        builder.addDoubleArrayProperty("Current pattern", ()->this.currentPattern, null);
    }

    public void ledPattern(double[] arr) {
        if (arr == null || arr.length < 2) return;

        long now = System.currentTimeMillis();

        if (ledPatternState == null) {
            ledPatternState = new long[]{now, 1, (long)(1000 / arr[0])};
            currentPattern = arr;
            currentValue = arr[1];
            return;
        }

        if (now - ledPatternState[0] >= ledPatternState[2]) {
            int index = (int) ledPatternState[1] + 1;
            if (index >= arr.length) index = 1;

            currentValue = arr[index];
            ledPatternState[0] = now;
            ledPatternState[1] = index;
        }
    }

    // Other methods go here
    public void setPattern(double[] pattern) {
        ledPatternState = null;
        currentPattern = pattern;
    }
    
    public Command stopMotor() {
        return runOnce(() -> LED.stopMotor());
    }

}
