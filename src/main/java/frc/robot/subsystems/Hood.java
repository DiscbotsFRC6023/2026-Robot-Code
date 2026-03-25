package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Ports;

import java.util.OptionalDouble;

public class Hood extends SubsystemBase {
    // Hood travel range in degrees (mechanical range of motion)
    // Adjust these based on your actual hood soft limits.
    private static final double kMinAngleDegrees = 0.0;
    private static final double kMaxAngleDegrees = 40.0;
    private static final double kPositionToleranceDegrees = 0.5; // acceptable error at the hood

    // Tunable shot range (degrees) — these can be live-updated via NetworkTables
    private static final double kDefaultInitialAngleDeg = 8.0;
    private static final double kDefaultMaxAngleDeg = 20.0;

    // AprilTag distance mapping (meters). Distances are clamped to this range before mapping to angles.
    // These roughly match the previous shot map breakpoints (~1.3m to ~4.2m).
    private static final double kMinTagDistanceMeters = 1.32;
    private static final double kMaxTagDistanceMeters = 4.20;

    // Limelight name as configured on the robot.
    private static final String kLimelightName = "limelight";

    // Gear ratio: 24T motor gear driving 36T hood gear
    // 1 motor rotation = (24/36) hood rotations = (24/36)*360 deg = 240 deg
    private static final double kDegreesPerMotorRotation = (24.0 / 36.0) * 360.0;

    // Motion Magic configuration (tune these for your mechanism)
    private static final double kCruiseVelocityRPS = 20.0; // rotations per second
    private static final double kAccelerationRPSPerSec = 40.0;

    private final TalonFX motor;

    // Tunable hood angle band for distance-based aiming
    private double hoodInitialAngleDeg = kDefaultInitialAngleDeg;
    private double hoodMaxAngleDeg = kDefaultMaxAngleDeg;

    // Current/target hood angle in degrees (for telemetry and logic)
    private double currentAngleDeg = 0.0;
    private double targetAngleDeg = 0.0;

    public Hood() {
        motor = new TalonFX(Ports.kHoodKrakenId, Ports.kCANivoreCANBus);

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = kCruiseVelocityRPS;
        mm.MotionMagicAcceleration = kAccelerationRPSPerSec;

        // Simple PID for position, tune as needed
        cfg.Slot0.kP = 40.0;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;

        StatusCode status = motor.getConfigurator().apply(cfg);
        if (!status.isOK()) {
            System.out.println("Hood TalonFX config error: " + status.toString());
        }

        // Reduce status frame rate to essentials
        BaseStatusSignal.setUpdateFrequencyForAll(50,
                motor.getPosition(),
                motor.getVelocity());

        // Initialize at the configured starting angle
        setAngleDegrees(hoodInitialAngleDeg);
        SmartDashboard.putData(this);
    }

    /**
     * Directly command a hood angle in degrees. Value is clamped to mechanical limits.
     */
    public void setAngleDegrees(double angleDegrees) {
        targetAngleDeg = MathUtil.clamp(angleDegrees, kMinAngleDegrees, kMaxAngleDegrees);

    // Convert hood angle (deg) to motor rotations
    double targetMotorRotations = targetAngleDeg / kDegreesPerMotorRotation;
    MotionMagicVoltage request = new MotionMagicVoltage(targetMotorRotations);
    motor.setControl(request);
    }

    /**
     * Set hood based on a normalized value between 0.0 (initial angle) and 1.0 (max angle).
     * This preserves existing call sites that were supplying normalized positions.
     */
    public void setNormalizedPosition(double normalized) {
        final double clamped = MathUtil.clamp(normalized, 0.0, 1.0);
        double desiredDeg = MathUtil.interpolate(hoodInitialAngleDeg, hoodMaxAngleDeg, clamped);
        setAngleDegrees(desiredDeg);
    }

    /**
     * Legacy alias retained for compatibility. Normalized position 0-1 maps to initial→max angles.
     */
    public void setPosition(double normalized) {
        setNormalizedPosition(normalized);
    }

    /**
     * Compute hood angle from an AprilTag distance and set it between the configured initial/max angles.
     * Distances are clamped to [kMinTagDistanceMeters, kMaxTagDistanceMeters].
     */
    public void setPositionFromAprilTagDistance(double distanceMeters) {
        double normalized = MathUtil.clamp(
            (distanceMeters - kMinTagDistanceMeters) / (kMaxTagDistanceMeters - kMinTagDistanceMeters),
            0.0,
            1.0
        );
        double desiredDeg = MathUtil.interpolate(hoodInitialAngleDeg, hoodMaxAngleDeg, normalized);
        setAngleDegrees(desiredDeg);
        SmartDashboard.putNumber("Hood/TagDistanceMeters", distanceMeters);
    }

    /**
     * Query the Limelight for the most recent AprilTag distance and position the hood accordingly.
     * If no tag is seen, the hood is left at its current setpoint.
     */
    public void setPositionFromLatestAprilTag() {
        OptionalDouble distanceMeters = getAprilTagDistanceMeters();
        if (distanceMeters.isPresent()) {
            setPositionFromAprilTagDistance(distanceMeters.getAsDouble());
        } else {
            SmartDashboard.putBoolean("Hood/TagVisible", false);
        }
    }

    private OptionalDouble getAprilTagDistanceMeters() {
        double[] targetPose = LimelightHelpers.getTargetPose_CameraSpace(kLimelightName);
        if (targetPose == null || targetPose.length < 3) {
            return OptionalDouble.empty();
        }

        // targetpose_cameraspace is [tx, ty, tz, roll, pitch, yaw] in meters/radians
        double x = targetPose[0];
        double y = targetPose[1];
        double z = targetPose[2];

        double distance = Math.sqrt(x * x + y * y + z * z);
        if (distance <= 0.0 || Double.isNaN(distance) || Double.isInfinite(distance)) {
            return OptionalDouble.empty();
        }

        SmartDashboard.putBoolean("Hood/TagVisible", true);
        return OptionalDouble.of(distance);
    }

    /**
     * Expects a normalized position between 0.0 (min angle) and 1.0 (max angle).
     */
    public Command positionCommand(double position) {
        return runOnce(() -> setNormalizedPosition(position))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public boolean isPositionWithinTolerance() {
    // Compute current hood angle from motor position
    double motorRotations = motor.getPosition().getValueAsDouble();
    currentAngleDeg = motorRotations * kDegreesPerMotorRotation;
    return MathUtil.isNear(targetAngleDeg, currentAngleDeg, kPositionToleranceDegrees);
    }

    @Override
    public void periodic() {
    // Track current angle for telemetry even if tolerance check isn't called
    double motorRotations = motor.getPosition().getValueAsDouble();
    currentAngleDeg = motorRotations * kDegreesPerMotorRotation;
    SmartDashboard.putNumber("Hood/CurrentAngle", currentAngleDeg);
    SmartDashboard.putNumber("Hood/MotorRotations", motorRotations);
    SmartDashboard.putNumber("Hood/CurrentAngleRotations", currentAngleDeg / 360.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Current Angle (deg)", () -> currentAngleDeg, null);
        builder.addDoubleProperty("Target Angle (deg)", () -> targetAngleDeg, this::setAngleDegrees);
        builder.addDoubleProperty("Initial Angle (deg)", () -> hoodInitialAngleDeg, value -> {
            hoodInitialAngleDeg = MathUtil.clamp(value, kMinAngleDegrees, hoodMaxAngleDeg);
        });
        builder.addDoubleProperty("Max Angle (deg)", () -> hoodMaxAngleDeg, value -> {
            hoodMaxAngleDeg = MathUtil.clamp(value, hoodInitialAngleDeg, kMaxAngleDegrees);
        });
        builder.addDoubleProperty("Min Tag Distance (m)", () -> kMinTagDistanceMeters, null);
        builder.addDoubleProperty("Max Tag Distance (m)", () -> kMaxTagDistanceMeters, null);
        builder.addDoubleProperty("Current Angle (rad)", () -> Units.degreesToRadians(currentAngleDeg), null);
    }
}