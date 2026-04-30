package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {
    private final String name;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;

    public Limelight(String name) {
        this.name = name;
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
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
        // Combination of MegaTag readings for better stability
        // Rotation std dev reduced to 30.0 to allow better rotation tracking and alignment
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.1, 0.1, 30.0);

        posePublisher.set(poseEstimate_MegaTag2.pose);

        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations));
    }

    public Optional<TargetObservation> getBestTargetObservation(int[] validTagIds) {
        RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(name);
        if (rawFiducials == null || rawFiducials.length == 0 || validTagIds == null || validTagIds.length == 0) {
            return Optional.empty();
        }

        TargetObservation best = null;
        for (RawFiducial fiducial : rawFiducials) {
            if (fiducial == null || !contains(validTagIds, fiducial.id)) {
                continue;
            }

            TargetObservation candidate = new TargetObservation(
                fiducial.id,
                fiducial.txnc,
                fiducial.tync,
                fiducial.ta,
                fiducial.distToRobot,
                fiducial.ambiguity
            );

            if (best == null || candidate.ambiguity() < best.ambiguity()) {
                best = candidate;
            }
        }

        return Optional.ofNullable(best);
    }

    private boolean contains(int[] values, int target) {
        return Arrays.stream(values).anyMatch(id -> id == target);
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }

    public static record TargetObservation(
        int id,
        double txDegrees,
        double tyDegrees,
        double area,
        double distanceMeters,
        double ambiguity
    ) {}
}