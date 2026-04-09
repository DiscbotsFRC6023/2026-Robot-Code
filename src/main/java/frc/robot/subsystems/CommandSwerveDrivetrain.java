package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
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
    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
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
        configureAutoBuilder();
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
        configureAutoBuilder();
    }

    // ======================== PATHPLANNER AUTO BUILDER ========================

    /**
     * Configures PathPlanner's AutoBuilder so it can generate path-following commands.
     * Call this once in the constructor after the drivetrain is fully initialized.
     */
    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose,                // Pose supplier
                this::resetOdometry,          // Pose reset consumer
                this::getChassisSpeeds,       // ChassisSpeeds supplier (robot-relative)
                (speeds, feedforwards) ->      // ChassisSpeeds consumer to drive the robot
                    drive(speeds.vxMetersPerSecond,
                          speeds.vyMetersPerSecond,
                          speeds.omegaRadiansPerSecond,
                          false),              // robot-relative since PathPlanner outputs robot-relative
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),   // Translation PID — tune these!
                    new PIDConstants(4.0, 0.0, 0.0)    // Rotation PID — tune these!
                ),
                config,
                () -> {
                    // Flip path if on Red alliance
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this  // Subsystem requirement
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to configure PathPlanner AutoBuilder: " + e.getMessage(), e.getStackTrace());
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

    /**
     * Locks the swerve modules in an "X" to resist pushing.
     * Runs continuously until another drive request is applied.
     */
    public Command xStanceCommand() {
        return applyRequest(() -> m_brake).withName("Swerve X Stance");
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
        quest.setPose(pose);
    }

    // ======================== COMMAND FACTORY ========================

    /**
     * Returns a command that applies the given swerve request continuously.
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }

    // ======================== QUEST NAVIGATION ========================

    private Quest quest;
    private double lastQuestTimestamp = -1.0;
    private final Matrix<N3, N1> questVisionStdDevs = VecBuilder.fill(0.1, 0.1, Math.toRadians(5.0));

    /**
     * Attach the QuestNav subsystem so its pose updates can be fused into the drivetrain odometry.
     */
    public void setQuest(Quest quest) {
        this.quest = quest;
    }

    @Override
    public void periodic() {
        if (quest != null) {
            double questTimestamp = quest.getLatestTimestamp();
            if (quest.hasFreshPose() && questTimestamp > lastQuestTimestamp) {
                addVisionMeasurement(quest.getLatestPose(), questTimestamp, questVisionStdDevs);
                lastQuestTimestamp = questTimestamp;
            }
        }
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
