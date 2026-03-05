package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import java.util.function.Supplier;

/**
 * Swerve drive subsystem built on CTRE Phoenix 6 swerve API.
 * Extends the Tuner X generated TunerSwerveDrivetrain and implements the WPILib Subsystem interface.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Reusable swerve request objects */
    private final SwerveRequest.FieldCentric m_fieldCentric = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric m_robotCentric = new SwerveRequest.RobotCentric();
    private final SwerveRequest.Idle m_idle = new SwerveRequest.Idle();

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency,
              odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    // ======================== DRIVE METHODS ========================

    /**
     * Drive the robot with the given velocities.
     *
     * @param vxMetersPerSecond     Forward velocity in m/s.
     * @param vyMetersPerSecond     Left velocity in m/s.
     * @param omegaRadPerSecond     Rotational velocity in rad/s.
     * @param fieldRelative         True for field-relative driving.
     */
    public void drive(double vxMetersPerSecond, double vyMetersPerSecond,
                      double omegaRadPerSecond, boolean fieldRelative) {
        if (fieldRelative) {
            setControl(m_fieldCentric
                .withVelocityX(vxMetersPerSecond)
                .withVelocityY(vyMetersPerSecond)
                .withRotationalRate(omegaRadPerSecond));
        } else {
            setControl(m_robotCentric
                .withVelocityX(vxMetersPerSecond)
                .withVelocityY(vyMetersPerSecond)
                .withRotationalRate(omegaRadPerSecond));
        }
    }

    /**
     * Stops all modules.
     */
    public void stop() {
        setControl(m_idle);
    }

    // ======================== GETTERS ========================

    /**
     * @return The estimated robot Pose2d from odometry.
     */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /**
     * @return Robot heading as a Rotation2d.
     */
    public Rotation2d getYaw() {
        return getPose().getRotation();
    }

    /**
     * @return The current chassis speeds (robot-relative).
     */
    public ChassisSpeeds getChassisSpeeds() {
        return getState().Speeds;
    }

    // ======================== GYRO & ODOMETRY ========================

    /**
     * Zeroes the field-centric heading.
     */
    public void zeroGyro() {
        seedFieldCentric();
    }

    /**
     * Resets the odometry to a given pose.
     */
    public void resetOdometry(Pose2d pose) {
        resetPose(pose);
    }

    // ======================== COMMAND FACTORY ========================

    /**
     * Returns a command that applies the given swerve request continuously.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }

    // ======================== SIMULATION ========================

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
