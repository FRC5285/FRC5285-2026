package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    // Instance variables go here
    
    public TurretSubsystem() {


        SendableRegistry.add(this, "Turret");
        SmartDashboard.putData(this);
    }

    // Other methods go here

    /**
     * The current turret angle, in rotations
     * 
     * @return the current turret angle, in rotations
     */
    public double turretAngle() {
        return 0.0;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        
    }
}
