package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Landmarks {
    private static final int[] RED_GOAL_TAG_IDS = {8, 5, 11, 2, 9, 10};
    private static final int[] BLUE_GOAL_TAG_IDS = {21, 24, 18, 27, 25, 26};

    public static Translation2d hubPosition() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(Inches.of(182.105), Inches.of(158.845));
        }
        return new Translation2d(Inches.of(469.115), Inches.of(158.845));
    }

    /**
     * Get the alignment target position based on alliance.
     * Red Alliance: x=11.928 m, y=4 m
     * Blue Alliance: x=4.641 m, y=4 m
     */
    public static Translation2d alignmentTarget() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return new Translation2d(4.641, 4.0);
        }
        return new Translation2d(11.928, 4.0);
    }

    public static int[] goalTagIds() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            return BLUE_GOAL_TAG_IDS;
        }
        return RED_GOAL_TAG_IDS;
    }
}