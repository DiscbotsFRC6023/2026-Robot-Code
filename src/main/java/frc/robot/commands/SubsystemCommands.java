package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Quest;
import frc.robot.subsystems.Shooter;

public final class SubsystemCommands {
    private final CommandSwerveDrivetrain swerve;
    private final Intake intake;
    private final Floor floor;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Hood hood;
    private final Hanger hanger;
    private final Limelight limelight;
    private final Quest quest;

    private final DoubleSupplier forwardInput;
    private final DoubleSupplier leftInput;

    public SubsystemCommands(
        CommandSwerveDrivetrain swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        Limelight limelight,
        Quest quest,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.floor = floor;
        this.feeder = feeder;
        this.shooter = shooter;
        this.hood = hood;
        this.hanger = hanger;
        this.limelight = limelight;
        this.quest = quest;

        this.forwardInput = forwardInput;
        this.leftInput = leftInput;
    }

    public SubsystemCommands(
        CommandSwerveDrivetrain swerve,
        Intake intake,
        Floor floor,
        Feeder feeder,
        Shooter shooter,
        Hood hood,
        Hanger hanger,
        Limelight limelight,
        Quest quest
    ) {
        this(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            limelight,
            quest,
            () -> 0,
            () -> 0
        );
    }

    public Command aimAndShoot() {
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, limelight, quest, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getPose());
        return Commands.parallel(
            aimAndDriveCommand,
            Commands.waitSeconds(0.25)
                .andThen(prepareShotCommand),
            Commands.race(
                Commands.waitUntil(() -> aimAndDriveCommand.isAimed() && prepareShotCommand.isReadyToShoot()),
                Commands.waitSeconds(0.75)
            ).andThen(
                Commands.waitSeconds(0.25),
                Commands.parallel(
                    feeder.feedCommand(),
                    Commands.waitSeconds(0.125)
                        .andThen(floor.feedCommand())
                )
            )
        );
    }

    /**
     * Command to align the robot to a target position using Quest-based positioning.
     * Uses manual translation controls while auto-aiming to face the alignment target.
     */
    public Command align() {
        return new Align(swerve, quest, forwardInput, leftInput);
    }

    public Command shootManually() {
        return shooter.spinUpCommand(1550.0)
            .andThen(
                Commands.waitSeconds(1),
                Commands.parallel(
                    feed()
                ))
            .handleInterrupt(() -> shooter.stop());
    }

    public Command ferry() {
        return shooter.spinUpCommand(2000.0)
            .andThen(
                Commands.waitSeconds(0),
                Commands.parallel(
                    feed()
                ))
            .handleInterrupt(() -> shooter.stop());
    }

    private Command feed() {
        return Commands.sequence(
            Commands.waitSeconds(0.25),
            Commands.parallel(
                feeder.feedCommand(),
                floor.feedCommand(),
                Commands.waitSeconds(2.5)
                    .andThen(intake.slowHomeCommand())
            )
        );
    }


    /**
     * Creates the default teleop swerve drive command.
     * Uses field-relative open-loop control with a configurable deadband.
     *
     * @param swerve       The swerve subsystem.
     * @param xSupplier    Left stick Y axis (forward/back). Negated because joystick Y is inverted.
     * @param ySupplier    Left stick X axis (strafe left/right).
     * @param rotSupplier  Right stick X axis (rotation).
     * @return A Command to run as the swerve default command.
     */
    public static Command teleopDrive(
            CommandSwerveDrivetrain swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier) {

        return Commands.run(() -> {
            /* Apply deadband */
            double xSpeed = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
            double ySpeed = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);
            double rotSpeed = MathUtil.applyDeadband(rotSupplier.getAsDouble(), 0.1);

            /* Square inputs for finer low-speed control */
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
            rotSpeed = Math.copySign(rotSpeed * rotSpeed, rotSpeed);

            /* Scale to m/s and rad/s */
            double translationX = xSpeed * Constants.Swerve.MAX_SPEED * Constants.Swerve.TELEOP_DRIVE_SPEED_MULTIPLIER;
            double translationY = ySpeed * Constants.Swerve.MAX_SPEED * Constants.Swerve.TELEOP_DRIVE_SPEED_MULTIPLIER;
            double rotation = rotSpeed * Constants.Swerve.MAX_ANGULAR_VELOCITY * Constants.Swerve.TELEOP_ROTATION_SPEED_MULTIPLIER;

            swerve.drive(translationX, translationY, rotation, true);
        }, swerve);
    }

    /**
     * Slow mode teleop drive command. Reduces swerve speed to 30% for precise control.
     * Only the swerve drivetrain moves slowly; other mechanisms are unaffected.
     * @param swerve       The swerve subsystem.
     * @param xSupplier    Left stick Y axis (forward/back). Negated because joystick Y is inverted.
     * @param ySupplier    Left stick X axis (strafe left/right).
     * @param rotSupplier  Right stick X axis (rotation).
     * @return A Command to run the swerve in slow mode.
     */
    public static Command teleopDriveSlow(
            CommandSwerveDrivetrain swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier) {

        final double SLOW_MODE_MULTIPLIER = 0.5;

        return Commands.run(() -> {
            /* Apply deadband */
            double xSpeed = MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1);
            double ySpeed = MathUtil.applyDeadband(ySupplier.getAsDouble(), 0.1);
            double rotSpeed = MathUtil.applyDeadband(rotSupplier.getAsDouble(), 0.1);

            /* Square inputs for finer low-speed control */
            xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
            ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
            rotSpeed = Math.copySign(rotSpeed * rotSpeed, rotSpeed);

            /* Scale to m/s and rad/s with slow mode multiplier */
            double translationX = xSpeed * Constants.Swerve.MAX_SPEED * Constants.Swerve.TELEOP_DRIVE_SPEED_MULTIPLIER * SLOW_MODE_MULTIPLIER;
            double translationY = ySpeed * Constants.Swerve.MAX_SPEED * Constants.Swerve.TELEOP_DRIVE_SPEED_MULTIPLIER * SLOW_MODE_MULTIPLIER;
            double rotation = rotSpeed * Constants.Swerve.MAX_ANGULAR_VELOCITY * Constants.Swerve.TELEOP_ROTATION_SPEED_MULTIPLIER * SLOW_MODE_MULTIPLIER;

            swerve.drive(translationX, translationY, rotation, true);
        }, swerve);
    }

    /**
     * Command to zero the gyro heading.
     */
    public static Command zeroGyro(CommandSwerveDrivetrain swerve) {
        return Commands.runOnce(swerve::zeroGyro, swerve).withName("Zero Gyro");
    }

    /**
     * Quest-based pose reset command.
     * Resets the drivetrain pose to the latest Quest pose at the start of autonomous.
     */
    public Command resetPoseToQuest() {
        return Commands.runOnce(() -> {
            if (quest.hasFreshPose()) {
                swerve.resetPose(quest.getLatestPose());
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Quest/Status", "Pose Reset to Quest");
            } else {
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Quest/Status", "Quest pose stale - reset skipped");
            }
        }, swerve).withName("Reset Pose to Quest");
    }

    /**
     * Quest correction guard command.
     * Monitors the pose deviation between Quest and swerve odometry during autonomous.
     * Logs warnings if deviation exceeds threshold.
     */
    public Command questCorrectionGuard() {
        return Commands.run(() -> {
            if (quest.hasFreshPose()) {
                var questPose = quest.getLatestPose();
                var swervePose = swerve.getPose();
                double positionDeviation = questPose.getTranslation().getDistance(swervePose.getTranslation());
                double rotationDeviation = Math.abs(questPose.getRotation().getDegrees() - swervePose.getRotation().getDegrees());
                
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Quest/Position Deviation (m)", positionDeviation);
                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("Quest/Rotation Deviation (deg)", rotationDeviation);
                
                // Log warning if deviation is too large (>0.5m)
                if (positionDeviation > 0.5) {
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Quest/Large Deviation Warning", true);
                    System.err.println("WARNING: Quest pose deviation > 0.5m: " + positionDeviation + "m");
                } else {
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Quest/Large Deviation Warning", false);
                }
            }
        }, swerve).withName("Quest Correction Guard");
    }

    /**
     * Mid-path Quest correction command.
     * If the robot drifts significantly in the Y direction (lateral), corrects pose based on Quest data.
     * Useful for correcting accumulated odometry drift during autonomous.
     */
    public Command midPathQuestCorrection() {
        return Commands.sequence(
            Commands.waitUntil(() -> quest.hasFreshPose()),
            Commands.runOnce(() -> {
                var questPose = quest.getLatestPose();
                var swervePose = swerve.getPose();
                
                // Only reset if lateral (Y) deviation is significant (>0.3m)
                double lateralDeviation = Math.abs(questPose.getY() - swervePose.getY());
                if (lateralDeviation > 0.3) {
                    swerve.resetPose(questPose);
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Quest/Status", "Mid-path correction applied");
                    System.out.println("Mid-path correction: lateral deviation was " + lateralDeviation + "m");
                } else {
                    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Quest/Status", "Mid-path correction not needed");
                }
            }, swerve)
        ).withTimeout(1).withName("Mid-Path Quest Correction");
    }

    // public Command shootByDistance() {
    //     return Commands.runOnce(() -> {
    //         var observation = limelight.getTargetObservation();
            
    //         double targetRPM = 1500.0; // Default RPM
            
    //         if (observation.isPresent()) {
    //             double distanceFeet = observation.get().distanceMeters() / 0.3048; // Convert to feet

    //             // Distance-based RPM lookup table with if-else statements
    //             if (distanceFeet <= 4.5) {
    //                 targetRPM = 1450.0;
    //             } else if (distanceFeet <= 5.5) {
    //                 targetRPM = 1500.0;
    //             } else if (distanceFeet <= 6.5) {
    //                 targetRPM = 1563.0;
    //             } else if (distanceFeet <= 7.5) {
    //                 targetRPM = 1610.0;
    //             } else if (distanceFeet <= 8.5) {
    //                 targetRPM = 1658.0;
    //             } else if (distanceFeet <= 9.5) {
    //                 targetRPM = 1703.0;
    //             } else if (distanceFeet <= 10.5) {
    //                 targetRPM = 1758.0;
    //             } else if (distanceFeet <= 11.5) {
    //                 targetRPM = 1815.0;
    //             } else if (distanceFeet >= 12.0) {
    //                 targetRPM = 1850.0;
    //             }
    //         }
            
    //         shooter.setDashboardTargetRPM(targetRPM);
    //     }).andThen(shooter.dashboardSpinUpCommand())
    //         .andThen(
    //             Commands.waitSeconds(0.75),
    //             Commands.parallel(
    //                 feed()
    //             ))
    //         .handleInterrupt(() -> shooter.stop());
    // }
}