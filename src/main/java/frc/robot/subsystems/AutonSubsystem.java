package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;

/** Handles autonomous, aimbot, and other commands */
public class AutonSubsystem extends SubsystemBase {

    private boolean shooting = false;
    private boolean climbing = false;

    private final CommandSwerveDrivetrain drivetrain;

    private PathConstraints climbPathConstraints = new PathConstraints(AutoConstants.climbMaxV, AutoConstants.climbMaxA, AutoConstants.climbMaxAngularV, AutoConstants.climbMaxAngularA);

    private SendableChooser<Integer> startPosition = new SendableChooser<>();
    private SendableChooser<Integer> route = new SendableChooser<>();
    private SendableChooser<Boolean> climbLeft = new SendableChooser<>();

    public AutonSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Start position
        this.startPosition.setDefaultOption("1", -2);
        for (int i = -1; i <= 2; i ++) this.startPosition.addOption("" + (i + 3), i);

        // Path
        this.route.setDefaultOption("Depot", 0);
        this.route.addOption("Outpost", 1);
        this.route.addOption("Left Neutral Zone", 2);
        this.route.addOption("Right Neutral Zone", 3);

        // Climb location
        this.climbLeft.setDefaultOption("Left", true);
        this.climbLeft.addOption("Right", false);

        // Puts choosers onto dashboard
        SmartDashboard.putData("Start Position", this.startPosition);
        SmartDashboard.putData("Where to get fuel", this.route);
        SmartDashboard.putData("Climb Position", this.climbLeft);
    }

    /**
     * If the shooter is trying to shoot
     * 
     * @return true if yes, false if no
     */
    public boolean isShooting() {
        return this.shooting;
    }

    /**
     * Try to shoot
     * 
     * @return Command turning shooting on
     */
    public Command shootingOn() {
        return runOnce(() -> {
            this.shooting = true;
        });
    }

    /**
     * Stop shooting
     * 
     * @return Command stopping shooting
     */
    public Command shootingOff() {
        return runOnce(() -> {
            this.shooting = false;
        });
    }

    /**
     * If a climb is happening
     * 
     * @return true if climbing, false if not
     */
    public boolean isClimbing() {
        return this.climbing;
    }

    /**
     * Climb the left side of the tower
     * 
     * @return A command climbing the left side of the tower
     */
    public Command climbLeft() {
        return runOnce(() -> {
            this.climbing = true;
        })
        .andThen(AutoBuilder.pathfindToPoseFlipped(FieldConstants.towerLeftPrepPose, this.climbPathConstraints))
        .andThen(this.drivetrain.fineTunePID(FieldConstants.towerLeftFinalPose))
        .andThen(runOnce(() -> {
            this.climbing = false;
        }));
    }

    /**
     * Climb the right side of the tower
     * 
     * @return A command climbing the right side of the tower
     */
    public Command climbRight() {
        return runOnce(() -> {
            this.climbing = true;
        })
        .andThen(AutoBuilder.pathfindToPoseFlipped(FieldConstants.towerRightPrepPose, this.climbPathConstraints))
        .andThen(this.drivetrain.fineTunePID(FieldConstants.towerRightFinalPose))
        .andThen(runOnce(() -> {
            this.climbing = false;
        }));
    }
}
