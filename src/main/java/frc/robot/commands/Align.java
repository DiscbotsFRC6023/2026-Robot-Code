package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Landmarks;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Quest;

/**
 * Drives the robot using manual translation inputs while automatically
 * rotating to face an alignment target position on the field.
 * Uses Quest for pose estimation.
 */
public class Align extends Command {
    /** Aim tolerance in degrees — if within this, {@link #isAimed()} returns true. */
    private static final double AIM_TOLERANCE_DEG = 5.0;

    private final CommandSwerveDrivetrain swerve;
    private final Quest quest;
    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    private final PIDController headingController;

    private Rotation2d targetHeading = new Rotation2d();

    public Align(
        CommandSwerveDrivetrain swerve,
        Quest quest,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.quest = quest;
        this.forwardInput = forwardInput;
        this.leftInput = leftInput;

        // PID controller for rotational heading (input/output in radians)
        // P-gain tuned for responsive aiming while preventing oscillation
        headingController = new PIDController(2, 0.0, 0.0);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(AIM_TOLERANCE_DEG));

        addRequirements(swerve);
    }

    public Align(CommandSwerveDrivetrain swerve, Quest quest) {
        this(swerve, quest, () -> 0, () -> 0);
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
     * Computes the field-relative direction from the robot to the alignment target.
     * The back of the robot faces the target.
     * Uses Quest pose estimate if available.
     */
    private Rotation2d getDirectionToTarget() {
        Translation2d robotPosition;
        boolean usingQuest = false;

        // Get current position from Quest
        if (quest.hasFreshPose()) {
            robotPosition = quest.getLatestPose().getTranslation();
            usingQuest = true;
        } else {
            // Fallback to swerve's internal odometry if Quest data is stale
            robotPosition = swerve.getPose().getTranslation();
        }

        SmartDashboard.putBoolean("Align/UsingQuest", usingQuest);
        SmartDashboard.putNumber("Align/RobotX", robotPosition.getX());
        SmartDashboard.putNumber("Align/RobotY", robotPosition.getY());

        Translation2d targetPosition = Landmarks.alignmentTarget();
        SmartDashboard.putNumber("Align/TargetX", targetPosition.getX());
        SmartDashboard.putNumber("Align/TargetY", targetPosition.getY());

        // Add 180 degrees so the back of the robot faces the target
        return targetPosition.minus(robotPosition).getAngle().plus(new Rotation2d(Math.PI));
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
        double headingError = headingController.calculate(currentYawRad, targetHeading.getRadians());
        
        // Scale PID output -to rad/s by multiplying by MAX_ANGULAR_VELOCITY
        double rotationOutput = headingError * Constants.Swerve.MAX_ANGULAR_VELOCITY;

        // Clamp rotation to max angular velocity
        rotationOutput = MathUtil.clamp(rotationOutput,
            -Constants.Swerve.MAX_ANGULAR_VELOCITY,
             Constants.Swerve.MAX_ANGULAR_VELOCITY);

        swerve.drive(translationX, translationY, rotationOutput, true);

        SmartDashboard.putNumber("Align/HeadingError", Math.toDegrees(targetHeading.getRadians() - currentYawRad));
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
