package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Landmarks;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Limelight;

/**
 * Drives the robot using manual translation inputs while automatically
 * rotating to face a target position on the field (e.g. the hub / speaker).
 * Uses PID-controlled heading with our custom Swerve subsystem.
 * Also controls the hood angle based on distance to target.
 */
public class AimAndDriveCommand extends Command {
    /** Aim tolerance in degrees — if within this, {@link #isAimed()} returns true. */
    private static final double AIM_TOLERANCE_DEG = 5.0;

    private static final InterpolatingTreeMap<Distance, Double> distanceToHoodPositionMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        Interpolator.forDouble()
    );

    static {
        // Hood position interpolation based on distance (same as PrepareShotCommand)
        distanceToHoodPositionMap.put(Inches.of(52.0), 0.19);
        distanceToHoodPositionMap.put(Inches.of(114.4), 0.40);
        distanceToHoodPositionMap.put(Inches.of(165.5), 0.48);
    }

    private final CommandSwerveDrivetrain swerve;
    private final Hood hood;
    private final Limelight limelight;
    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    private final PIDController headingController;

    private Rotation2d targetHeading = new Rotation2d();

    public AimAndDriveCommand(
        CommandSwerveDrivetrain swerve,
        Hood hood,
        Limelight limelight,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.hood = hood;
        this.limelight = limelight;
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;

        // PID controller for rotational heading (input/output in radians)
        headingController = new PIDController(5.0, 0.0, 0.0);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(AIM_TOLERANCE_DEG));

        addRequirements(swerve, hood);
    }

    public AimAndDriveCommand(CommandSwerveDrivetrain swerve, Hood hood, Limelight limelight) {
        this(swerve, hood, limelight, () -> 0, () -> 0);
    }

    /**
     * @return true if the robot's heading is within tolerance of the target direction.
     */
    public boolean isAimed() {
        double currentRad = swerve.getYaw().getRadians();
        double targetRad = targetHeading.getRadians();
        double error = Math.abs(MathUtil.angleModulus(targetRad - currentRad));
        return error < Math.toRadians(AIM_TOLERANCE_DEG);
    }

    /**
     * Computes the field-relative direction from the robot to the target.
     * Uses Limelight pose estimate if available, otherwise falls back to landmark position.
     */
    private Rotation2d getDirectionToTarget() {
        Translation2d robotPosition = swerve.getPose().getTranslation();
        
        // Try to get updated pose from Limelight for better AprilTag-based alignment
        var measurement = limelight.getMeasurement(swerve.getPose());
        if (measurement.isPresent()) {
            robotPosition = measurement.get().poseEstimate.pose.getTranslation();
            SmartDashboard.putBoolean("AimAndDrive/UsingLimelight", true);
        } else {
            SmartDashboard.putBoolean("AimAndDrive/UsingLimelight", false);
        }
        
        return Landmarks.hubPosition().minus(robotPosition).getAngle();
    }

    /**
     * Gets the distance from the robot to the target hub.
     */
    private Distance getDistanceToHub() {
        Translation2d robotPosition = swerve.getPose().getTranslation();
        Translation2d hubPosition = Landmarks.hubPosition();
        return Meters.of(robotPosition.getDistance(hubPosition));
    }

    @Override
    public void initialize() {
        headingController.reset();
    }

    @Override
    public void execute() {
        // --- Translation (manual input, same processing as teleopDrive) ---
        double xSpeed = MathUtil.applyDeadband(forwardInput.getAsDouble(), 0.1);
        double ySpeed = MathUtil.applyDeadband(leftInput.getAsDouble(), 0.1);

        // Square for finer low-speed control
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);

        double translationX = xSpeed * Constants.Swerve.MAX_SPEED * Constants.Swerve.TELEOP_DRIVE_SPEED_MULTIPLIER;
        double translationY = ySpeed * Constants.Swerve.MAX_SPEED * Constants.Swerve.TELEOP_DRIVE_SPEED_MULTIPLIER;

        // --- Rotation (auto-aim via PID) ---
        targetHeading = getDirectionToTarget();
        double currentYawRad = swerve.getYaw().getRadians();
        double rotationOutput = headingController.calculate(currentYawRad, targetHeading.getRadians());

        // Clamp rotation to max angular velocity
        rotationOutput = MathUtil.clamp(rotationOutput,
            -Constants.Swerve.MAX_ANGULAR_VELOCITY,
             Constants.Swerve.MAX_ANGULAR_VELOCITY);

        swerve.drive(translationX, translationY, rotationOutput, true);

        // --- Hood positioning based on distance to target ---
        Distance distanceToHub = getDistanceToHub();
        double hoodPosition = distanceToHoodPositionMap.get(distanceToHub);
        hood.setPosition(hoodPosition);
        
        SmartDashboard.putNumber("AimAndDrive/DistanceToHub", distanceToHub.in(Inches));
        SmartDashboard.putNumber("AimAndDrive/HoodPosition", hoodPosition);
        SmartDashboard.putNumber("AimAndDrive/HeadingError", Math.toDegrees(targetHeading.getRadians() - currentYawRad));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}