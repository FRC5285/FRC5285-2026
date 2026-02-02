package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.generated.TunerConstants;

public final class Constants {
    /** Constants relating to the driver input */
    public static class OperatorConstants {
        /** The driver station port to use as the driver controller port */
        public static final int driverControllerPort = 0;

        /** The deadband for the driver controller joysticks */
        public static final double driveDeadband = 0.1;

        /** Max robot speed, in meters per second */
        public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        /** Max robot rotation rate, in radians per second (not the real one, just the driver limit) */
        public static final double maxAngularRate = 2 * Math.PI;
    }

    /** Constants for use during Auto */
    public static class AutoConstants {

    }

    /** Positions of stuff on the field, Welded field */
    public static class FieldConstants {

    }

    /** Constants for vision */
    public static class VisionConstants {
        /** The field tag layout */
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        /** Standard deviations for when only one tag is seen */
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.5);
        /** Standard deviations for when multiple tags are seen */
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 0.2);

        /** Number of cameras on the robot */
        public static final int numCameras = 4;

        /** Camera names */
        public static final String[] cameraNames = {
            "Arducam_OV9281_USB1",
            "Arducam_OV9281_USB2",
            "Arducam_OV9281_USB3",
            "Arducam_OV9281_USB4"
        };

        /** Camera positions on the robot */
        public static final Transform3d[] cameraOffsets = {
            new Transform3d(),
            new Transform3d(),
            new Transform3d(),
            new Transform3d()
        };
    }

    /** Constants for turret */
    public static class TurretConstants {

    }

    /** Constants for intake */
    public static class IntakeConstants {

    }

    /** Constants for storage rollers */
    public static class RollerConstants {

    }

    /** Constants for climber */
    public static class ClimbConstants {

    }

    /** Constants for LEDs */
    public static class LedConstants {

    }
}
