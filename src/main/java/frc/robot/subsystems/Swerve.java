package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Swerve drive subsystem using 4× TalonFX (Falcon 500) modules with CANcoders
 * and a Pigeon 2 gyro. Built on Phoenix 6.
 */
public class Swerve extends SubsystemBase {
    private final SwerveModule[] modules;
    private final Pigeon2 gyro;
    private final SwerveDriveOdometry odometry;
    private final Field2d field;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.PIGEON_ID);
        gyro.reset();

        modules = new SwerveModule[] {
            new SwerveModule(0,
                Constants.Swerve.FL_DRIVE_MOTOR_ID,
                Constants.Swerve.FL_ANGLE_MOTOR_ID,
                Constants.Swerve.FL_CANCODER_ID,
                Constants.Swerve.FL_ANGLE_OFFSET),
            new SwerveModule(1,
                Constants.Swerve.FR_DRIVE_MOTOR_ID,
                Constants.Swerve.FR_ANGLE_MOTOR_ID,
                Constants.Swerve.FR_CANCODER_ID,
                Constants.Swerve.FR_ANGLE_OFFSET),
            new SwerveModule(2,
                Constants.Swerve.BL_DRIVE_MOTOR_ID,
                Constants.Swerve.BL_ANGLE_MOTOR_ID,
                Constants.Swerve.BL_CANCODER_ID,
                Constants.Swerve.BL_ANGLE_OFFSET),
            new SwerveModule(3,
                Constants.Swerve.BR_DRIVE_MOTOR_ID,
                Constants.Swerve.BR_ANGLE_MOTOR_ID,
                Constants.Swerve.BR_CANCODER_ID,
                Constants.Swerve.BR_ANGLE_OFFSET)
        };

        odometry = new SwerveDriveOdometry(
            Constants.Swerve.SWERVE_KINEMATICS,
            getYaw(),
            getModulePositions()
        );

        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    // ======================== DRIVE METHODS ========================

    /**
     * Main drive method — called from commands.
     *
     * @param translation   Desired X/Y speeds in m/s (robot or field-relative).
     * @param rotation      Desired rotational speed in rad/s.
     * @param fieldRelative True for field-relative driving.
     * @param isOpenLoop    True for open-loop voltage control (teleop), false for closed-loop velocity (auto).
     */
    public void drive(
            double translationX,
            double translationY,
            double rotation,
            boolean fieldRelative,
            boolean isOpenLoop) {

        ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translationX, translationY, rotation, getYaw())
                : new ChassisSpeeds(translationX, translationY, rotation);

        SwerveModuleState[] moduleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.MAX_SPEED);

        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(moduleStates[i], isOpenLoop);
        }
    }

    /**
     * Directly set the module states (used by SwerveControllerCommand / PathPlanner).
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(desiredStates[i], false);
        }
    }

    /**
     * Stops all modules.
     */
    public void stop() {
        drive(0, 0, 0, false, true);
    }

    // ======================== GYRO ========================

    /**
     * @return Robot heading as a Rotation2d (CCW-positive).
     */
    public Rotation2d getYaw() {
        return gyro.getRotation2d();
    }

    /**
     * Zeroes the gyro heading.
     */
    public void zeroGyro() {
        gyro.reset();
    }

    /**
     * Sets the gyro heading to a specific angle.
     */
    public void setGyro(double degrees) {
        gyro.setYaw(degrees);
    }

    // ======================== ODOMETRY ========================

    /**
     * @return The estimated robot Pose2d from odometry.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to a given pose.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * @return The current chassis speeds (robot-relative).
     */
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    // ======================== MODULE STATE HELPERS ========================

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    // ======================== PERIODIC ========================

    @Override
    public void periodic() {
        odometry.update(getYaw(), getModulePositions());
        field.setRobotPose(getPose());

        // Per-module telemetry
        for (SwerveModule mod : modules) {
            mod.putSmartDashboard();
        }

        SmartDashboard.putNumber("Swerve/Gyro Yaw", getYaw().getDegrees());
        SmartDashboard.putString("Swerve/Pose", getPose().toString());
    }
}
