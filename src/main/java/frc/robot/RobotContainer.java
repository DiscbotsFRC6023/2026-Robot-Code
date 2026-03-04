// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);

  /* Subsystems */
  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();
  private final Floor floor = new Floor();
  private final Feeder feeder = new Feeder();
  private final Shooter shooter = new Shooter();
  private final Hood hood = new Hood();
  private final Hanger hanger = new Hanger();



  public RobotContainer() {
    /* Set default commands */
    swerve.setDefaultCommand(
        SubsystemCommands.teleopDrive(
            swerve,
            () -> -driver.getLeftY(),  // forward/back (negate because stick Y is inverted)
            () -> -driver.getLeftX(),  // strafe left/right
            () -> -driver.getRightX() // rotation
        )

    );

    configureBindings();
  }

  private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );

  private void configureBindings() {
    driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
    driver.rightBumper().whileTrue(subsystemCommands.shootManually());
    driver.leftTrigger().whileTrue(intake.intakeCommand());
    driver.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));

    /* Back button → zero gyro heading */
    driver.back().onTrue(SubsystemCommands.zeroGyro(swerve));

    /* Start button → zero gyro heading (alternative) */
    driver.start().onTrue(SubsystemCommands.zeroGyro(swerve));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /* Expose subsystems if needed */
  public Swerve getSwerve() {
    return swerve;
  }

  /** Returns the driver controller HID so Robot can set rumble. */
  public GenericHID getDriverController() {
    return driver.getHID();
  }
}
