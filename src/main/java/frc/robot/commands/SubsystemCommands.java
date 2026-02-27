package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SubsystemCommands {

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
            Swerve swerve,
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

            swerve.drive(translationX, translationY, rotation, true, true);
        }, swerve);
    }

    /**
     * Command to zero the gyro heading.
     */
    public static Command zeroGyro(Swerve swerve) {
        return Commands.runOnce(swerve::zeroGyro, swerve).withName("Zero Gyro");
    }
}
