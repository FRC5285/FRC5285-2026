package frc.robot.subsystems;

import org.wpilib.drivers.motor.Spark;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import frc.robot.util.ShiftUtil;
import frc.robot.Constants.LEDConstants;

//global usage, only one pattern at a time, 
//theLED.currentPattern = trans_flag; //first index is speed in Hz


public class LedSubsystem extends SubsystemBase {
    private final Spark LED = new Spark(LEDConstants.led_pin);
    public long[] ledPatternState = null;
    public double[] currentPattern = null;
    private double currentValue = 0.0;

    public Command hubActive() {
        return runOnce(() -> {
            // green
            currentPattern = LEDConstants.aro_flag;
        });
    }

    public Command hubInactive() {
        return runOnce(() -> {
            currentPattern = LEDConstants.bisexual_flag;
        });
    }

    public Command preHub() {
        return runOnce(() -> {
            currentPattern = LEDConstants.blink_orange;
        });
    }

    public Command auton() {
        return runOnce(() -> {
            // orange0[-9ob ]
            currentPattern = LEDConstants.lesbian_flag;
        });
    }

    public Command shootFailed() {
        return runOnce(() -> {
            // red
            currentPattern = LEDConstants.queer_flag;
        });
    }

    @Override
    public void periodic() {
        if (currentPattern != null) {
            ledPattern(currentPattern);
            LED.setThrottle(currentValue);
        }
    }

    @Override
    public void logTo(TelemetryTable table) {
        table.log("Can score", ShiftUtil.canScore());
        table.log("Current pattern", this.currentPattern);
    }

    // ----------------------------------------------------------------------------------------------- //

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
