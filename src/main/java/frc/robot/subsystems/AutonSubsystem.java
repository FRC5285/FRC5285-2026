package frc.robot.subsystems;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.PositionMath;
import frc.robot.util.ShiftUtil;

/** Handles autonomous, aimbot, and other commands */
public class AutonSubsystem extends SubsystemBase {

    private boolean shooting = false;
    private boolean climbing = false;

    private final CommandSwerveDrivetrain drivetrain;
    private final IntakeSubsystem groundIntake;
    private final TurretIntakeSubsystem turretIntake;
    private final RollerSubsystem bucketRollers;
    private final BucketOutSubsystem bucketOuttake;
    private final ClimbSubsystem climber;
    private final LedSubsystem ledSubsystem;
    private final PositionMath positionMath;

    private PathConstraints autonPathConstraints = new PathConstraints(AutoConstants.maxV, AutoConstants.maxA, AutoConstants.maxAngularV, AutoConstants.maxAngularA);
    // private PathConstraints climbPathConstraints = new PathConstraints(AutoConstants.climbMaxV, AutoConstants.climbMaxA, AutoConstants.climbMaxAngularV, AutoConstants.climbMaxAngularA);

    private SendableChooser<Integer> startPosition = new SendableChooser<>();
    private SendableChooser<Supplier<Command>> collectLocation = new SendableChooser<>();
    private SendableChooser<Supplier<Command>> climbCommand = new SendableChooser<>();

    public AutonSubsystem(CommandSwerveDrivetrain drivetrain, IntakeSubsystem groundIntake, TurretIntakeSubsystem turretIntake, RollerSubsystem bucketRollers, BucketOutSubsystem bucketOuttake, ClimbSubsystem climber, LedSubsystem ledSubsystem, PositionMath positionMath) {
        this.drivetrain = drivetrain;
        this.groundIntake = groundIntake;
        this.turretIntake = turretIntake;
        this.bucketRollers = bucketRollers;
        this.bucketOuttake = bucketOuttake;
        this.climber = climber;
        this.ledSubsystem = ledSubsystem;
        this.positionMath = positionMath;

        // Start position
        this.startPosition.setDefaultOption("1", -2);
        for (int i = -1; i <= 2; i ++) this.startPosition.addOption("" + (i + 3), i);

        // Path
        this.collectLocation.setDefaultOption("Depot (Left)", () -> this.depotCollection());
        this.collectLocation.addOption("Outpost (Right)", () -> this.outpostCollection());
        this.collectLocation.addOption("Left Neutral Zone", () -> this.leftNeutralZoneCollection());
        this.collectLocation.addOption("Right Neutral Zone", () -> this.rightNeutralZoneCollection());

        // Climb location
        this.climbCommand.setDefaultOption("Left", () -> this.climbLeft(false));
        this.climbCommand.addOption("Right", () -> this.climbRight(false));

        // Puts choosers onto dashboard
        SmartDashboard.putData("Start Position", this.startPosition);
        SmartDashboard.putData("Where to get fuel", this.collectLocation);
        SmartDashboard.putData("Climb Position", this.climbCommand);
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
        })
        .andThen(this.turretIntake.beginIntake())
        .andThen(new WaitUntilCommand(() -> this.turretIntake.atTargetSpeed()).withTimeout(AutoConstants.turretIntakeMaxWaitTime))
        .andThen(this.bucketOuttake.startCommand())
        .andThen(this.bucketRollers.startFastCommand());
    }

    /**
     * Stop shooting
     * 
     * @return Command stopping shooting
     */
    public Command shootingOff() {
        return runOnce(() -> {
            this.shooting = false;
        })
        .andThen(this.bucketRollers.stopCommand())
        // .andThen(this.turretIntake.reverseIntake())
        // .andThen(this.bucketOuttake.setReverse())
        ;
    }

    /** Call a couple seconds after shootingOff() */
    public Command shootingOffStop() {
        return this.turretIntake.endIntake()
        .andThen(this.bucketOuttake.stopCommand());
    }

    /** Full command to turn off shooting */
    public Command shootingOffFull() {
        return this.shootingOff()
        .andThen(new WaitCommand(0.0))
        .andThen(this.shootingOffStop());
    }

    /** command to regurgitate the balls from the bucket */
    public Command regurgitate() {
        return this.turretIntake.reverseIntake()
        .andThen(this.bucketOuttake.setReverse())
        .andThen(this.bucketRollers.reverseCommand())
        .andThen(this.groundIntake.reverseIntake());
    }
    
    /** command to end regurgitation */
    public Command stopRegurgitate() {
        return this.bucketOuttake.stopCommand()
        // .andThen(this.bucketRollers.startCommand())
        ;
    }

    /** Move the ground intake down (begin intaking) */
    public Command intakeDown() {
        return this.groundIntake.lowerIntake()
        // .andThen(this.groundIntake.beginIntake())
        // .andThen(this.bucketRollers.startCommand())
        ;
    }

    /** Move the ground intake up (stop intaking) */
    public Command intakeUp() {
        return this.groundIntake.endIntake()
        .andThen(this.bucketRollers.stopCommand())
        .andThen(this.groundIntake.raiseIntake());
    }

    /** What the LEDs should show when not trying to shoot */
    private Command ledsCommand() {
        if (ShiftUtil.canScore()) {
            return this.ledSubsystem.hubActive();
        } else if (ShiftUtil.beforeShooting()) {
            return this.ledSubsystem.preHub();
        }
        return this.ledSubsystem.hubInactive();
    }

    /**
     * The command for the auton
     * 
     * @return the auton command
     */
    public Command autonCommand() {
        return runOnce(() -> {
            this.drivetrain.resetPose(this.positionMath.drivetrainStartPosition(this.startPosition.getSelected()));
        })
        .alongWith(this.ledSubsystem.auton())
        .andThen(AutoBuilder.pathfindToPoseFlipped(this.autonInitialShootPose(this.startPosition.getSelected()), this.autonPathConstraints))
        .andThen(this.shootingOn())
        .andThen(new WaitCommand(4.0))
        .andThen(this.shootingOffFull())
        .andThen(this.intakeDown())
        .andThen(this.groundIntake.beginIntake())
        .andThen(this.collectLocation.getSelected().get())
        .andThen(this.groundIntake.endIntake())
        .andThen(this.climbCommand.getSelected().get());
    }

    private Pose2d autonInitialShootPose(int startPoseNumber) {
        return startPoseNumber <= 0 ? FieldConstants.blueLeftShootPose : FieldConstants.blueRightShootPose;
    }

    /**
     * Auton to collect fuel from the depot and shoot it
     * 
     * @return A command to collect fuel from the depot and shoot it
     */
    public Command depotCollection() {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile("Depot Collection");
        } catch (Exception e) {
            path = null;
        }
        return AutoBuilder.pathfindThenFollowPath(path, autonPathConstraints)
        .andThen(this.intakeUp())
        .andThen(this.shootingOn())
        .andThen(new WaitCommand(AutoConstants.shootTime));
    }

    /**
     * Auton to collect fuel from the outpost and shoot it
     * 
     * @return A command to collect fuel from the outpost and shoot it
     */
    public Command outpostCollection() {
        return AutoBuilder.pathfindToPoseFlipped(FieldConstants.blueOutpostParkPose, autonPathConstraints)
        .andThen(new WaitCommand(AutoConstants.outpostWaitTime))
        .andThen(AutoBuilder.pathfindToPoseFlipped(FieldConstants.blueOutpostShootPose, autonPathConstraints))
        .andThen(this.intakeUp())
        .andThen(this.shootingOn())
        .andThen(new WaitCommand(AutoConstants.shootTime));
    }

    /**
     * Auton to collect fuel from the left of the neutral zone and shoot it
     * 
     * @return a command to do the auton
     */
    public Command leftNeutralZoneCollection() {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile("Neutral Zone Left");
        } catch (Exception e) {
            path = null;
        }
        return this.neutralZoneCollection(path);
    }

    /**
     * Auton to collect fuel from the right of the neutral zone and shoot it
     * 
     * @return a command to do the auton
     */
    public Command rightNeutralZoneCollection() {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile("Neutral Zone Left").mirrorPath();
        } catch (Exception e) {
            path = null;
        }
        return this.neutralZoneCollection(path);
    }

    /**
     * Auton to collect fuel using a path going to the neutral zone and then shoot
     * 
     * @param path the path to follow
     * @return the auton command
     */
    public Command neutralZoneCollection(PathPlannerPath path) {
        return AutoBuilder.pathfindThenFollowPath(path, autonPathConstraints)
        .andThen(this.intakeUp())
        .andThen(this.shootingOn())
        .andThen(new WaitCommand(AutoConstants.shootMoreTime));
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
     * @param inTeleop do the teleop climb?
     * @return A command climbing the left side of the tower
     */
    public Command climbLeft(boolean inTeleop) {
        return runOnce(() -> {
            this.climbing = true;
        })
        // .alongWith(this.ledSubsystem.auton())
        // .alongWith(this.intakeUp())
        // .andThen(this.shootingOff())
        // .andThen(AutoBuilder.pathfindToPoseFlipped(FieldConstants.towerLeftPrepPose, this.climbPathConstraints))
        // .andThen(this.shootingOffStop())
        // .andThen(this.drivetrain.fineTunePID(FieldConstants.towerLeftFinalPose))
        // .andThen(inTeleop ? this.climber.Climb() : this.climber.AutonClimb())
        .andThen(this.shootingOffFull())
        .andThen(runOnce(() -> {
            this.climbing = false;
            // this.positionMath.resetLastRotation();
        }))
        .andThen(this.ledsCommand());
    }

    /**
     * Climb the right side of the tower
     * 
     * @param inTeleop do the teleop climb?
     * @return A command climbing the right side of the tower
     */
    public Command climbRight(boolean inTeleop) {
        return runOnce(() -> {
            this.climbing = true;
        })
        // .alongWith(this.ledSubsystem.auton())
        // .alongWith(this.intakeUp())
        // .andThen(this.shootingOff())
        // .andThen(AutoBuilder.pathfindToPoseFlipped(FieldConstants.towerRightPrepPose, this.climbPathConstraints))
        // .andThen(this.shootingOffStop())
        // .andThen(this.drivetrain.fineTunePID(FieldConstants.towerRightFinalPose))
        // .andThen(inTeleop ? this.climber.Climb() : this.climber.AutonClimb())
        .andThen(this.shootingOffFull())
        .andThen(runOnce(() -> {
            this.climbing = false;
            // this.positionMath.resetLastRotation();
        }))
        .andThen(this.ledsCommand());
    }

    public Command teleopStartCommand() {
        return this.climber.Unclimb();
    }
}
