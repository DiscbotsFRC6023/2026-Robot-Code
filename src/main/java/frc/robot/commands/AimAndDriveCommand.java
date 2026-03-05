package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Landmarks;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Drives the robot using manual translation inputs while automatically
 * rotating to face a target position on the field (e.g. the hub / speaker).
 * Uses PID-controlled heading with our custom Swerve subsystem.
 */
public class AimAndDriveCommand extends Command {
    /** Aim tolerance in degrees — if within this, {@link #isAimed()} returns true. */
    private static final double AIM_TOLERANCE_DEG = 5.0;

    private final CommandSwerveDrivetrain swerve;
    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    private final PIDController headingController;

    private Rotation2d targetHeading = new Rotation2d();

    public AimAndDriveCommand(
        CommandSwerveDrivetrain swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;

        // PID controller for rotational heading (input/output in radians)
        headingController = new PIDController(5.0, 0.0, 0.0);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(AIM_TOLERANCE_DEG));

        addRequirements(swerve);
    }

    public AimAndDriveCommand(CommandSwerveDrivetrain swerve) {
        this(swerve, () -> 0, () -> 0);
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
     */
    private Rotation2d getDirectionToTarget() {
        Translation2d robotPosition = swerve.getPose().getTranslation();
        return Landmarks.hubPosition().minus(robotPosition).getAngle();
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