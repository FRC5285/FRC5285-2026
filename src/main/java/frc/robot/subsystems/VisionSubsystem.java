package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.PositionMath;

// "Heavy inspiration" was taken from the following source:
// https://github.com/PhotonVision/photonvision/blob/main/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java

/** A class for the vision subsystem. Go to localhost:[camera port] for the simulated camera view. */
public class VisionSubsystem extends SubsystemBase {
    private final PositionMath positionMath;

    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] photonEstimators;
    private Matrix<N3, N1> curStdDevs;
    private final Map<Integer, TimeInterpolatableBuffer<Pose3d>> camTrfMap = new HashMap<>();

    private final EstimateConsumer estConsumer;
    private final Supplier<Pose2d> robotPoseSupplier;

    private VisionSystemSim visionSim;
    private PhotonCameraSim[] camerasSim;

    public VisionSubsystem(EstimateConsumer estimateConsumer, Supplier<Pose2d> robotPose, PositionMath positionMath) {
        this.positionMath = positionMath;
        // Add pose to drivetrain method
        this.estConsumer = estimateConsumer;
        // Robot pose supplier
        this.robotPoseSupplier = robotPose;

        // Sets up cameras and pose estimators
        this.cameras = new PhotonCamera[VisionConstants.numCameras];
        this.photonEstimators = new PhotonPoseEstimator[VisionConstants.numCameras];
        for (int i = 0; i < VisionConstants.numCameras; i ++) {
            this.cameras[i] = new PhotonCamera(VisionConstants.cameraNames[i]);
            this.photonEstimators[i] = new PhotonPoseEstimator(VisionConstants.kTagLayout, this.getCurrentRobotCameraTransform(i));
            this.camTrfMap.put(i, TimeInterpolatableBuffer.createBuffer(VisionConstants.camPositionBufferTime));
            this.camTrfMap.get(i).addSample(i, new Pose3d().plus(this.getCurrentRobotCameraTransform(i)));
        }

        // Simulation start code
        if (Robot.isSimulation()) {
            this.simInit();
        }

        SendableRegistry.add(this, "Vision");
        SmartDashboard.putData(this);
    }

    /** Inits simulation variables */
    private void simInit() {
        // Create the vision system simulation which handles cameras and targets on the field.
        this.visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
        this.visionSim.addAprilTags(VisionConstants.kTagLayout);
        // Create simulated camera properties. These can be set to mimic your actual camera.
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.5, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);
        cameraProp.setLatencyStdDevMs(15);
        // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        // targets.
        this.camerasSim = new PhotonCameraSim[VisionConstants.numCameras];
        for (int i = 0; i < VisionConstants.numCameras; i ++) {
            this.camerasSim[i] = new PhotonCameraSim(this.cameras[i], cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            this.visionSim.addCamera(this.camerasSim[i], VisionConstants.cameraOffsets[i]);
    
            this.camerasSim[i].enableDrawWireframe(true);
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < VisionConstants.numCameras; i ++) {
            this.setRobotCameraTransform(i);
            this.updateRobotPose(i);
        }
    }

    @Override
    public void simulationPeriodic() {
        for (int i = 0; i < VisionConstants.numCameras; i ++) {
            this.simSetRobotCameraTransform(i);
        }
        this.visionSim.update(robotPoseSupplier.get());
    }

    /** Updates the robot pose with data from a specified camera
     * 
     * @param cameraIndex the camera to use
     */
    private void updateRobotPose(int cameraIndex) {
        for (var result : this.cameras[cameraIndex].getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            this.photonEstimators[cameraIndex].setRobotToCameraTransform(this.getRobotCameraTransform(cameraIndex, result.getTimestampSeconds()));
            // Get pose estimation (through multitag)
            visionEst = this.photonEstimators[cameraIndex].estimateCoprocMultiTagPose(result);
            // If only one tag visible
            if (visionEst.isEmpty()) {
                visionEst = this.photonEstimators[cameraIndex].estimateLowestAmbiguityPose(result);
            }
            // Update standard deviations
            this.updateEstimationStdDevs(cameraIndex, visionEst, result.getTargets());

            // Sim stuff
            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                    est -> this.getSimDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
                    () -> this.getSimDebugField().getObject("VisionEstimation").setPoses()
                );
            }

            // Apply estimated pose to drivetrain
            visionEst.ifPresent(est -> this.estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, this.getEstimationStdDevs()));
        }
    }

    /** ALWAYS CALL {@link VisionSubsystem#updateEstimationStdDevs(int, Optional, List)} RIGHT BEFORE USING!!!!!! 
     * 
     * @return the standard deviations for the pose estimation
    */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return this.curStdDevs;
    }

    /** Updates the position possible deviation based off how many tags the camera can see and the distance of the tags */
    private void updateEstimationStdDevs(int cameraIndex, Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimators[cameraIndex].getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags ++;
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 3.0)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + avgDist * avgDist);
                curStdDevs = estStdDevs;
            }
        }
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) this.visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return this.visionSim.getDebugField();
    }

    /**
     * Gets the transform of a camera at the current time
     * 
     * @param cameraIndex the camera
     * @return the transform of the camera right now
     */
    public Transform3d getCurrentRobotCameraTransform(int cameraIndex) {
        if (VisionConstants.turretMounted[cameraIndex]) {
            return this.positionMath.getCameraTurretTransform(VisionConstants.cameraOffsets[cameraIndex]);
        }
        return VisionConstants.cameraOffsets[cameraIndex];
    }

    /**
     * Puts the camera transform into the interpolation buffer
     * 
     * @param cameraIndex the camera
     */
    private void setRobotCameraTransform(int cameraIndex) {
        this.camTrfMap.get(cameraIndex).addSample(Timer.getFPGATimestamp(), new Pose3d().plus(this.getCurrentRobotCameraTransform(cameraIndex)));
    }

    /**
     * Gets the camera transform at a timestamp
     * 
     * @param cameraIndex the camera
     * @param timestampFPGA the time (FPGA time)
     * @return the transform at the timestamp
     */
    public Transform3d getRobotCameraTransform(int cameraIndex, double timestampFPGA) {
        return new Transform3d(new Pose3d(), this.camTrfMap.get(cameraIndex).getSample(timestampFPGA).orElse(new Pose3d().plus(this.getCurrentRobotCameraTransform(cameraIndex))));
    }

    private void simSetRobotCameraTransform(int cameraIndex) {
        this.visionSim.adjustCamera(this.camerasSim[cameraIndex], this.getCurrentRobotCameraTransform(cameraIndex));
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

}
