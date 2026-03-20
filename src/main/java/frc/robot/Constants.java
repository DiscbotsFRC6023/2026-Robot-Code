package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;

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
        /* Max speeds derived from TunerConstants */
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MAX_ANGULAR_VELOCITY = MAX_SPEED
                / Math.hypot(Units.inchesToMeters(11.5555), Units.inchesToMeters(10.125)); // rad/s

        /* Teleop Drive Multipliers */
        public static final double TELEOP_DRIVE_SPEED_MULTIPLIER = 1.0;
        public static final double TELEOP_ROTATION_SPEED_MULTIPLIER = 1.0;
    }

    public static class Limelight {

    }

    public static class Quest {

    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }
}
