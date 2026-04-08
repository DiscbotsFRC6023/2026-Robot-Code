package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;

    /** Camera mounting info. */
    //private static final double CAMERA_PITCH_DEGREES = 28.1;
    //private static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(22.32);
    //private static final double CAMERA_LATERAL_OFFSET_METERS = Units.inchesToMeters(5.74); // +Y is left of robot

    /** Red alliance: aim at tag 10 (9 and 10 are side-by-side). */
    private static final int RED_TARGET_TAG_ID = 10;
    /** Blue alliance: aim at tag 25 (25 and 26 are side-by-side). */
    private static final int BLUE_TARGET_TAG_ID = 25;

    /** Tracks which alliance filter is currently applied so we don't spam NT. */
    private Alliance lastAlliance = null;

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();

        // Tell Limelight where the camera is on the robot so pose solves are correct.
        // LimelightHelpers.setCameraPose_RobotSpace(
        //     name,
        //     0.0,                              // forward offset (unknown/assumed 0)
        //     //CAMERA_LATERAL_OFFSET_METERS,     // left of robot center
        //     //CAMERA_HEIGHT_METERS,             // height from carpet to lens
        //     0.0,                              // roll
        //     //CAMERA_PITCH_DEGREES,             // pitch up
        //     0.0                               // yaw
        // );
    }

    /**
     * Updates the AprilTag ID filter based on the current alliance colour.
     * Red → tag 10, Blue → tag 25.  Called every cycle so the filter is
     * correct even if alliance info arrives late from the Driver Station.
     */
    private void updateTagFilter() {
        final Alliance alliance = DriverStation.getAlliance().orElse(null);
        if (alliance == null || alliance == lastAlliance) {
            return; // nothing to change (or unknown yet)
        }
        lastAlliance = alliance;

        int targetTag = (alliance == Alliance.Blue) ? BLUE_TARGET_TAG_ID : RED_TARGET_TAG_ID;
        LimelightHelpers.SetFiducialIDFiltersOverride(name, new int[]{ targetTag });
        LimelightHelpers.setPriorityTagID(name, targetTag);
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        // Make sure we're filtering for the correct alliance's AprilTag
        updateTagFilter();

        LimelightHelpers.SetRobotOrientation(name, currentRobotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);

        final PoseEstimate poseEstimate_MegaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        final PoseEstimate poseEstimate_MegaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (
            poseEstimate_MegaTag1 == null 
                || poseEstimate_MegaTag2 == null
                || poseEstimate_MegaTag1.tagCount == 0
                || poseEstimate_MegaTag2.tagCount == 0
        ) {
            return Optional.empty();
        }

        // Combine the readings from MegaTag1 and MegaTag2:
        // 1. Use the more stable position from MegaTag2
        // 2. Use the rotation from MegaTag1 (with low confidence) to counteract gyro drift
        poseEstimate_MegaTag2.pose = new Pose2d(
            poseEstimate_MegaTag2.pose.getTranslation(),
            poseEstimate_MegaTag1.pose.getRotation()
        );
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.1, 0.1, 10.0);

        posePublisher.set(poseEstimate_MegaTag2.pose);

        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations));
    }

    /**
     * Returns the current target observation relative to the robot using camera-space pose.
     * Distance/Yaw are corrected for the camera's lateral offset so callers can drive from the robot center.
     */
    public Optional<TargetObservation> getTargetObservation() {
        updateTagFilter();

        double[] targetPoseCameraSpace = LimelightHelpers.getTargetPose_CameraSpace(name);
        if (targetPoseCameraSpace == null || targetPoseCameraSpace.length < 3 || !LimelightHelpers.getTV(name)) {
            return Optional.empty();
        }

        // Limelight camera-space: +X forward, +Y left, +Z up (meters)
        final double cameraX = targetPoseCameraSpace[0];
        final double cameraY = targetPoseCameraSpace[1];
        final double cameraZ = targetPoseCameraSpace[2];

        // Shift from camera origin to robot origin using the known lateral offset (no yaw offset assumed)
        final double robotX = cameraX;
        //final double robotY = cameraY + CAMERA_LATERAL_OFFSET_METERS;
        final double robotY = cameraY;

        final double distanceMeters = Math.hypot(robotX, robotY);
        final double distanceFeet = Units.metersToFeet(distanceMeters);
        // Calculate yaw error relative to robot center
        // atan2(Y, X) gives angle from X-axis toward Y
        // Since +Y is left and we want positive yaw to mean "rotate counter-clockwise to face target",
        // we negate the result to align with robot heading convention
        final double yawRadians = -Math.atan2(robotY, robotX);

        telemetryTable.getEntry("Tag Distance (m)").setDouble(distanceMeters);
        telemetryTable.getEntry("Tag Distance (ft)").setDouble(distanceFeet);
        telemetryTable.getEntry("Tag Yaw (deg)").setDouble(Math.toDegrees(yawRadians));

        return Optional.of(new TargetObservation(distanceMeters, yawRadians, robotX, robotY, cameraZ));
    }

    public static record TargetObservation(
        double distanceMeters,
        double yawRadians,
        double xMeters,
        double yMeters,
        double zMeters
    ) {}

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }
}