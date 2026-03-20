package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }
}