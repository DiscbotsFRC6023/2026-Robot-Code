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
import frc.robot.subsystems.Align;
import frc.robot.subsystems.Limelight;
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
    private final Align align;

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
        Align align,
        Limelight limelight,
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
        this.align = align;
        this.limelight = limelight;

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
        Align align,
        Limelight limelight
    ) {
        this(
            swerve,
            intake,
            floor,
            feeder,
            shooter,
            hood,
            hanger,
            align,
            limelight,
            () -> 0,
            () -> 0
        );
    }

    public Command aimAndShoot() {
        final AimAndDriveCommand aimAndDriveCommand = new AimAndDriveCommand(swerve, limelight, forwardInput, leftInput);
        final PrepareShotCommand prepareShotCommand = new PrepareShotCommand(shooter, hood, () -> swerve.getPose());
        return Commands.parallel(
            aimAndDriveCommand,
            Commands.waitSeconds(0.25)
                .andThen(prepareShotCommand),
            intake.slowHomeCommand(),
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

    public Command shootManually() {
        return shooter.spinUpCommand(1500.0)
            .andThen(
                Commands.waitSeconds(0.75),
                Commands.parallel(
                    feed()
                ))
            .handleInterrupt(() -> shooter.stop());
    }

    /** Align to the alliance speaker tag using Limelight and shoot while held. */
    public Command limelightAimAndShoot() {
        final Command alignCommand = align.alignCommand();

        final Command spinUpShooter = shooter.spinUpCommand(3000);

        final Command feedWhenReady = Commands.sequence(
            Commands.waitUntil(() -> align.isAligned() && shooter.isVelocityWithinTolerance()),
            Commands.waitSeconds(0.1),
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125).andThen(floor.feedCommand())
            )
        );

        return Commands.parallel(
            alignCommand,
            spinUpShooter,
            intake.slowHomeCommand(),
            feedWhenReady
        ).finallyDo(() -> shooter.stop());
    }

    private Command feed() {
        return Commands.sequence(
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125)
                    .andThen(floor.feedCommand().alongWith(intake.agitateCommand()))
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

        final double SLOW_MODE_MULTIPLIER = 0.3;

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

    public Command shootByDistance() {
        return Commands.runOnce(() -> {
            var observation = limelight.getTargetObservation();
            
            double targetRPM = 1500.0; // Default RPM
            
            if (observation.isPresent()) {
                double distanceFeet = observation.get().distanceMeters() / 0.3048; // Convert to feet

                // Distance-based RPM lookup table with if-else statements
                if (distanceFeet <= 4.5) {
                    targetRPM = 1450.0;
                } else if (distanceFeet <= 5.5) {
                    targetRPM = 1500.0;
                } else if (distanceFeet <= 6.5) {
                    targetRPM = 1563.0;
                } else if (distanceFeet <= 7.5) {
                    targetRPM = 1610.0;
                } else if (distanceFeet <= 8.5) {
                    targetRPM = 1658.0;
                } else if (distanceFeet <= 9.5) {
                    targetRPM = 1703.0;
                } else if (distanceFeet <= 10.5) {
                    targetRPM = 1758.0;
                } else if (distanceFeet <= 11.5) {
                    targetRPM = 1815.0;
                } else if (distanceFeet >= 12.0) {
                    targetRPM = 1850.0;
                }
            }
            
            shooter.setDashboardTargetRPM(targetRPM);
        }).andThen(shooter.dashboardSpinUpCommand())
            .andThen(
                Commands.waitSeconds(0.75),
                Commands.parallel(
                    feed()
                ))
            .handleInterrupt(() -> shooter.stop());
    }
}