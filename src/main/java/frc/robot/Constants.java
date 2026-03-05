package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public final class Constants {
    public static class Shooter {

    }

    public static class Climber {

    }

    public static class Intake {

    }

    public static class Feeder {
        public static final double intakeSpeed = 1;

    }

    public static class Swerve {
        /* Gyro */
        public static final int PIGEON_ID = 13;

        /* Drivetrain Constants */
        // Distance between left and right wheels (meters)
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.73);
        // Distance between front and back wheels (meters)
        public static final double WHEEL_BASE = Units.inchesToMeters(21.73);
        // Wheel diameter in meters (4 inch colson / MK4i standard)
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        /*
         * Gear ratios for SDS MK4i modules.
         * Adjust these if using a different module or gear ratio.
         * MK4i L2: drive = 6.75:1, steer = 150/7 : 1
         */
        public static final double DRIVE_GEAR_RATIO = 6.75;
        public static final double ANGLE_GEAR_RATIO = 150.0 / 7.0;

        /* Drive Motor Conversion Factors */
        // rotor rotations → meters
        public static final double DRIVE_ROTOR_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
        // rotor RPS → m/s
        public static final double DRIVE_ROTOR_RPS_TO_MPS = DRIVE_ROTOR_TO_METERS;

        /* Angle Motor Conversion Factor */
        // rotor rotations → mechanism rotations (wheel angle)
        public static final double ANGLE_ROTOR_TO_ROTATIONS = 1.0 / ANGLE_GEAR_RATIO;

        /* Kinematics */
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),   // Front Left
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),  // Front Right
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),  // Back Left
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)  // Back Right
        );

        /* Max Speed & Acceleration */
        // Falcon 500 free speed: ~6380 RPM → ~106.3 RPS
        public static final double MAX_SPEED = (6380.0 / 60.0) * DRIVE_ROTOR_TO_METERS; // ~4.5 m/s for L2
        public static final double MAX_ANGULAR_VELOCITY = MAX_SPEED
                / Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0); // rad/s

        /* Drive Motor PID & FF */
        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KS = 0.05;
        public static final double DRIVE_KV = 0.12;

        /* Angle Motor PID */
        public static final double ANGLE_KP = 50.0;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.5;

        /* Current Limits */
        public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 40.0;
        public static final double ANGLE_SUPPLY_CURRENT_LIMIT = 25.0;

        /* Neutral Modes */
        // true = brake, false = coast
        public static final boolean DRIVE_BRAKE_MODE = true;
        public static final boolean ANGLE_BRAKE_MODE = true;

        /* Motor & Encoder Inversions */
        public static final boolean DRIVE_MOTOR_INVERTED = false;
        public static final boolean ANGLE_MOTOR_INVERTED = true; // MK4i steer is inverted
        public static final boolean CANCODERS_INVERTED = false;

        /* Open-Loop Ramp Rate (seconds from 0 to full output) */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Teleop Drive Multipliers */
        public static final double TELEOP_DRIVE_SPEED_MULTIPLIER = 1.0;
        public static final double TELEOP_ROTATION_SPEED_MULTIPLIER = 1.0;

        /*
         * ==================== MODULE-SPECIFIC CONSTANTS ====================
         * Update CAN IDs and CANcoder magnet offsets to match YOUR robot.
         * Magnet offsets are in ROTATIONS (0 to 1). Use Phoenix Tuner X to find them.
         */

        /* Front Left Module - Module 0 */
        public static final int FL_DRIVE_MOTOR_ID = 1;
        public static final int FL_ANGLE_MOTOR_ID = 2;
        public static final int FL_CANCODER_ID = 3;
        public static final double FL_ANGLE_OFFSET = 0.0; // TODO: Set with Phoenix Tuner X

        /* Front Right Module - Module 1 */
        public static final int FR_DRIVE_MOTOR_ID = 4;
        public static final int FR_ANGLE_MOTOR_ID = 5;
        public static final int FR_CANCODER_ID = 6;
        public static final double FR_ANGLE_OFFSET = 0.0; // TODO: Set with Phoenix Tuner X

        /* Back Left Module - Module 2 */
        public static final int BL_DRIVE_MOTOR_ID = 7;
        public static final int BL_ANGLE_MOTOR_ID = 8;
        public static final int BL_CANCODER_ID = 9;
        public static final double BL_ANGLE_OFFSET = 0.0; // TODO: Set with Phoenix Tuner X

        /* Back Right Module - Module 3 */
        public static final int BR_DRIVE_MOTOR_ID = 10;
        public static final int BR_ANGLE_MOTOR_ID = 11;
        public static final int BR_CANCODER_ID = 12;
        public static final double BR_ANGLE_OFFSET = 0.0; // TODO: Set with Phoenix Tuner X
    }

    public static class Limelight {

    }

    public static class Quest {

    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }
}
