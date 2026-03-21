package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

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
import frc.robot.Ports;

public class Hood extends SubsystemBase {
    // Hood travel range in degrees (mechanical range of motion)
    // Adjust these based on your actual hood soft limits.
    private static final double kMinAngleDegrees = 0.0;
    private static final double kMaxAngleDegrees = 40.0;
    private static final double kPositionToleranceDegrees = 0.5; // acceptable error at the hood

    // Gear ratio: motor rotations per hood degree
    // Example: 50 motor rotations per 1 hood rotation (360 deg) -> 50/360 per degree.
    // Tune this to match your mechanism.
    private static final double kMotorRotationsPerHoodDegree = 50.0 / 360.0;

    // Motion Magic configuration (tune these for your mechanism)
    private static final double kCruiseVelocityRPS = 20.0; // rotations per second
    private static final double kAccelerationRPSPerSec = 40.0;

    private final TalonFX motor;

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

        // Initialize at min angle
        setPosition(0.0);
        SmartDashboard.putData(this);
    }

    /**
     * Expects a normalized position between 0.0 (min angle) and 1.0 (max angle).
     */
    public void setPosition(double position) {
        final double clampedPosition = MathUtil.clamp(position, 0.0, 1.0);

        // Convert normalized 0-1 to hood angle in degrees
        targetAngleDeg = MathUtil.interpolate(kMinAngleDegrees, kMaxAngleDegrees, clampedPosition);

        // Convert hood angle (deg) to motor rotations
        double targetMotorRotations = targetAngleDeg * kMotorRotationsPerHoodDegree;

        MotionMagicVoltage request = new MotionMagicVoltage(targetMotorRotations);
        motor.setControl(request);
    }

    /**
     * Expects a normalized position between 0.0 (min angle) and 1.0 (max angle).
     */
    public Command positionCommand(double position) {
        return runOnce(() -> setPosition(position))
            .andThen(Commands.waitUntil(this::isPositionWithinTolerance));
    }

    public boolean isPositionWithinTolerance() {
        // Compute current hood angle from motor position
        double motorRotations = motor.getPosition().getValueAsDouble();
        currentAngleDeg = motorRotations / kMotorRotationsPerHoodDegree;

        return MathUtil.isNear(targetAngleDeg, currentAngleDeg, kPositionToleranceDegrees);
    }

    @Override
    public void periodic() {
        // Nothing required; closed-loop handled by TalonFX Motion Magic
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Current Angle (deg)", () -> currentAngleDeg, null);
        builder.addDoubleProperty("Target Angle (deg)", () -> targetAngleDeg, value -> {
            // Allow live tuning of target angle in degrees from dashboard
            targetAngleDeg = MathUtil.clamp(value, kMinAngleDegrees, kMaxAngleDegrees);
            double targetMotorRotations = targetAngleDeg * kMotorRotationsPerHoodDegree;
            motor.setControl(new MotionMagicVoltage(targetMotorRotations));
        });
        builder.addDoubleProperty("Current Angle (rad)", () -> Units.degreesToRadians(currentAngleDeg), null);
    }
}