package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;

public final class Constants {
    public static class Shooter {
        public static final int ID_columMotorLeft = 0; // TODO: SET THE CAN ID
        public static final int ID_columMotorCenter = 0; // TODO: SET THE CAN ID
        public static final int ID_columMotorRight = 0; // TODO: SET THE CAN ID
    }

    public static class Climber {
        public static final int ID_climberMotor = 0; // TODO: SET THE CAN ID
    }

    public static class Intake {
        public static final int ID_intakeMotor = 0; // TODO: SET THE CAN ID
    }

    public static class Feeder {
        public static final double intakeSpeed = 1;

        public static final int ID_feederMotor = 0; // TODO: SET THE CAN ID

    }

    public static class Swerve {
        /* Max speeds derived from TunerConstants */
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MAX_ANGULAR_VELOCITY = MAX_SPEED
                / Math.hypot(Units.inchesToMeters(11.5555), Units.inchesToMeters(10.125)); // rad/s

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
        public static final int FL_CANCODER_ID = 01;
        public static final double FL_ANGLE_OFFSET = 0.0; // TODO: Set with Phoenix Tuner X

        /* Front Right Module - Module 1 */
        public static final int FR_DRIVE_MOTOR_ID = 4;
        public static final int FR_ANGLE_MOTOR_ID = 5;
        public static final int FR_CANCODER_ID = 02;
        public static final double FR_ANGLE_OFFSET = 0.0; // TODO: Set with Phoenix Tuner X

        /* Back Left Module - Module 2 */
        public static final int BL_DRIVE_MOTOR_ID = 10;
        public static final int BL_ANGLE_MOTOR_ID = 11;
        public static final int BL_CANCODER_ID = 03;
        public static final double BL_ANGLE_OFFSET = 0.0; // TODO: Set with Phoenix Tuner X

        /* Back Right Module - Module 3 */
        public static final int BR_DRIVE_MOTOR_ID = 7;
        public static final int BR_ANGLE_MOTOR_ID = 8;
        public static final int BR_CANCODER_ID = 4;
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
