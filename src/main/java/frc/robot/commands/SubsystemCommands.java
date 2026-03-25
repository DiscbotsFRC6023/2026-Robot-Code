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
        return Commands.parallel(
            shooter.dashboardSpinUpCommand(),
            Commands.run(() -> hood.setAngleDegrees(7.0), hood)
        )
            .andThen(Commands.waitSeconds(0.5))
            .andThen(
                Commands.parallel(
                    feed(),
                    Commands.run(() -> hood.setAngleDegrees(7.0), hood),
                    Commands.waitSeconds(4.0).andThen(intake.slowHomeCommand())
                ))
            .handleInterrupt(() -> shooter.stop());
    }

    /** Align to the alliance speaker tag using Limelight, set hood from distance, and shoot while held. */
    public Command limelightAimAndShoot() {
        final Command alignCommand = align.alignCommand();

        // Get hood position from Limelight directly and continuously hold it
        final Command hoodFromDistance = Commands.run(
            () -> {
                limelight.getTargetObservation().ifPresent(obs -> hood.setPositionFromAprilTagDistance(obs.distanceMeters()));
                // Continuously apply the current target position to fight gravity
                hood.setAngleDegrees(hood.getTargetAngleDegrees());
            },
            hood,
            limelight
        );

        final Command spinUpShooter = shooter.spinUpCommand(4500);

        final Command feedWhenReady = Commands.sequence(
            Commands.waitUntil(() -> align.isAligned() && shooter.isVelocityWithinTolerance() && hood.isPositionWithinTolerance()),
            Commands.waitSeconds(0.1),
            Commands.parallel(
                feeder.feedCommand(),
                Commands.waitSeconds(0.125).andThen(floor.feedCommand())
            )
        );

        return Commands.parallel(
            alignCommand,
            hoodFromDistance,
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
                    .andThen(floor.feedCommand().alongWith(intake.slowHomeCommand()))
            )
        );
    }

    /**
     * Creates a command that continuously holds the hood at its current target position.
     * This applies a periodic re-command to fight gravity sag. 
     * Useful for holding hood position during extended operations.
     */
    public Command hoodHoldCommand() {
        return Commands.run(() -> hood.setAngleDegrees(hood.getTargetAngleDegrees()), hood)
            .withName("Hood Hold");
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
     * Command to zero the gyro heading.
     */
    public static Command zeroGyro(CommandSwerveDrivetrain swerve) {
        return Commands.runOnce(swerve::zeroGyro, swerve).withName("Zero Gyro");
    }
}
