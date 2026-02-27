package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * A single swerve module with a TalonFX drive motor, TalonFX steer motor, and CANcoder.
 * Uses Phoenix 6 API.
 */
public class SwerveModule {
    private final int moduleNumber;
    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private final CANcoder canCoder;

    /* Control Requests */
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut driveOpenLoop = new VoltageOut(0);
    private final PositionVoltage anglePosition = new PositionVoltage(0).withSlot(0);

    private Rotation2d lastAngle = new Rotation2d();

    /**
     * Creates a new SwerveModule.
     *
     * @param moduleNumber  Module index (0 = FL, 1 = FR, 2 = BL, 3 = BR)
     * @param driveMotorID  CAN ID of the drive TalonFX
     * @param angleMotorID  CAN ID of the steer TalonFX
     * @param canCoderID    CAN ID of the CANcoder
     * @param angleOffset   CANcoder magnet offset in rotations
     */
    public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int canCoderID, double angleOffset) {
        this.moduleNumber = moduleNumber;

        /* CANcoder Configuration */
        canCoder = new CANcoder(canCoderID);
        configCANCoder(angleOffset);

        /* Angle (Steer) Motor Configuration */
        angleMotor = new TalonFX(angleMotorID);
        configAngleMotor();

        /* Drive Motor Configuration */
        driveMotor = new TalonFX(driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    // ======================== CONFIGURATION ========================

    private void configCANCoder(double angleOffset) {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = Constants.Swerve.CANCODERS_INVERTED
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = angleOffset;
        canCoder.getConfigurator().apply(config);
    }

    private void configAngleMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        /* Motor output */
        config.MotorOutput.Inverted = Constants.Swerve.ANGLE_MOTOR_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = Constants.Swerve.ANGLE_BRAKE_MODE
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

        /* Current Limits */
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.ANGLE_SUPPLY_CURRENT_LIMIT;

        /* PID — slot 0 */
        config.Slot0.kP = Constants.Swerve.ANGLE_KP;
        config.Slot0.kI = Constants.Swerve.ANGLE_KI;
        config.Slot0.kD = Constants.Swerve.ANGLE_KD;

        /* Use the remote CANcoder as feedback so we're controlling in absolute rotations */
        config.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.RotorToSensorRatio = Constants.Swerve.ANGLE_GEAR_RATIO;
        config.Feedback.SensorToMechanismRatio = 1.0;

        /* Enable continuous wrap so the module takes the shortest path */
        config.ClosedLoopGeneral.ContinuousWrap = true;

        angleMotor.getConfigurator().apply(config);
    }

    private void configDriveMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        /* Motor output */
        config.MotorOutput.Inverted = Constants.Swerve.DRIVE_MOTOR_INVERTED
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = Constants.Swerve.DRIVE_BRAKE_MODE
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

        /* Current Limits */
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.DRIVE_SUPPLY_CURRENT_LIMIT;

        /* PID & Feedforward — slot 0 */
        config.Slot0.kP = Constants.Swerve.DRIVE_KP;
        config.Slot0.kI = Constants.Swerve.DRIVE_KI;
        config.Slot0.kD = Constants.Swerve.DRIVE_KD;
        config.Slot0.kS = Constants.Swerve.DRIVE_KS;
        config.Slot0.kV = Constants.Swerve.DRIVE_KV;

        /* Open-loop ramp for teleop feel */
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.OPEN_LOOP_RAMP;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.CLOSED_LOOP_RAMP;

        driveMotor.getConfigurator().apply(config);
        driveMotor.setPosition(0.0);
    }

    // ======================== SETTERS ========================

    /**
     * Sets the desired state for this swerve module.
     *
     * @param desiredState The desired {@link SwerveModuleState}.
     * @param isOpenLoop   True for open-loop (teleop), false for closed-loop (auto).
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Optimize the state so the module doesn't rotate >90° when it can just reverse drive
        desiredState.optimize(getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent jittering when speed is nearly zero
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01))
                ? lastAngle
                : desiredState.angle;

        // Position is in rotations (Phoenix 6 native unit)
        angleMotor.setControl(anglePosition.withPosition(angle.getRotations()));
        lastAngle = angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            // Scale desired speed to voltage (-12 to 12)
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            driveMotor.setControl(driveOpenLoop.withOutput(percentOutput * 12.0));
        } else {
            // Convert m/s → rotor RPS
            double velocityRPS = desiredState.speedMetersPerSecond / Constants.Swerve.DRIVE_ROTOR_TO_METERS;
            driveMotor.setControl(driveVelocity.withVelocity(velocityRPS));
        }
    }

    // ======================== GETTERS ========================

    /**
     * @return The current angle of the module as a Rotation2d (from CANcoder absolute position).
     */
    public Rotation2d getCANCoderAngle() {
        return Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * @return The current angle of the module as reported by the steer motor (fused CANcoder).
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
    }

    /**
     * @return The current velocity of the drive motor in m/s.
     */
    public double getDriveVelocity() {
        // TalonFX velocity is in rotor RPS; convert to m/s
        return driveMotor.getVelocity().getValueAsDouble() * Constants.Swerve.DRIVE_ROTOR_TO_METERS;
    }

    /**
     * @return The total distance driven by this module in meters.
     */
    public double getDrivePosition() {
        // TalonFX position is in rotor rotations; convert to meters
        return driveMotor.getPosition().getValueAsDouble() * Constants.Swerve.DRIVE_ROTOR_TO_METERS;
    }

    /**
     * @return The current {@link SwerveModuleState} (velocity + angle).
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAngle());
    }

    /**
     * @return The current {@link SwerveModulePosition} (distance + angle).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getAngle());
    }

    /**
     * @return The module number (0-3).
     */
    public int getModuleNumber() {
        return moduleNumber;
    }

    // ======================== TELEMETRY ========================

    public void putSmartDashboard() {
        SmartDashboard.putNumber("Swerve/Mod " + moduleNumber + "/CANcoder Angle", getCANCoderAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Mod " + moduleNumber + "/Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Mod " + moduleNumber + "/Velocity", getDriveVelocity());
        SmartDashboard.putNumber("Swerve/Mod " + moduleNumber + "/Position", getDrivePosition());
    }
}
