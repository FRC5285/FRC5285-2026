package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    // Instance variables go here
    // ethan k
    
    public LedSubsystem() {


        SendableRegistry.add(this, "LEDs");
        SmartDashboard.putData(this);
    }

    // Other methods go here


    @Override
    public void periodic() {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        
    }
}
