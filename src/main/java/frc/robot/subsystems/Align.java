package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem that rotates the robot to face the alliance-specific speaker AprilTag
 * (red tag 10, blue tag 25). Uses Limelight target observations and the swerve drivetrain.
 */
public class Align extends SubsystemBase {
    private static final double ALIGN_TOLERANCE_RAD = Math.toRadians(2.0);
    private static final double ROTATION_KP = 5.0;

    private final CommandSwerveDrivetrain swerve;
    private final Limelight limelight;
    private final PIDController rotationController = new PIDController(ROTATION_KP, 0.0, 0.0);

    private Optional<Limelight.TargetObservation> latestObservation = Optional.empty();

    public Align(CommandSwerveDrivetrain swerve, Limelight limelight) {
        this.swerve = swerve;
        this.limelight = limelight;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(ALIGN_TOLERANCE_RAD);
        rotationController.setSetpoint(0.0);
    }

    private void updateObservation() {
        final Optional<Limelight.TargetObservation> observation = limelight.getTargetObservation();
        if (observation.isPresent()) {
            latestObservation = observation;
        }
    }

    public Optional<Limelight.TargetObservation> getLatestObservation() {
        return latestObservation;
    }

    public OptionalDouble getLatestDistanceMeters() {
        return latestObservation.map(o -> OptionalDouble.of(o.distanceMeters())).orElse(OptionalDouble.empty());
    }

    public boolean isAligned() {
        return rotationController.atSetpoint();
    }

    /** Perform one alignment step: rotate toward the target if visible, otherwise stop. */
    public void alignStep() {
        updateObservation();

        if (latestObservation.isEmpty()) {
            rotationController.reset();
            swerve.stop();
            return;
        }

        final double yawError = latestObservation.get().yawRadians();
        double rotationOutput = rotationController.calculate(yawError, 0.0);
        rotationOutput = MathUtil.clamp(rotationOutput,
            -Constants.Swerve.MAX_ANGULAR_VELOCITY,
             Constants.Swerve.MAX_ANGULAR_VELOCITY);

        swerve.drive(0.0, 0.0, rotationOutput, true);
    }

    /** Command wrapper so callers can schedule alignment easily. */
    public Command alignCommand() {
        return Commands.run(this::alignStep, swerve, this).withName("AlignToAprilTag");
    }
}
