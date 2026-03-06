// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SubsystemCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
  private final CommandXboxController aux    = new CommandXboxController(1);

  /* Subsystems */
  private final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
  private final Intake intake = new Intake();
  private final Floor floor = new Floor();
  private final Feeder feeder = new Feeder();
  private final Shooter shooter = new Shooter();
  private final Hood hood = new Hood();
  private final Hanger hanger = new Hanger();
  private final Limelight limelight = new Limelight("limelight");



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
        limelight,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );

  private void configureBindings() {
    /* ── Driver (port 0) – swerve only ── */

    /* Back button → zero gyro heading */
    driver.back().onTrue(SubsystemCommands.zeroGyro(swerve));

    /* Start button → zero gyro heading (alternative) */
    driver.start().onTrue(SubsystemCommands.zeroGyro(swerve));

    /* X button → zero gyro heading */
    driver.x().onTrue(SubsystemCommands.zeroGyro(swerve));

    /* ── Aux (port 1) – mechanisms ── */
    aux.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
    aux.rightBumper().whileTrue(subsystemCommands.shootManually());
    aux.leftTrigger().whileTrue(intake.intakeCommand());
    aux.leftBumper().onTrue(intake.runOnce(() -> intake.handleLeftBumperPress(true)));
  }

  public Command getAutonomousCommand() {
    // Simple auton: spin up shooters, then run feeders + floor to push game pieces through
    return Commands.sequence(
        // 1. Spin up the shooter and wait until it's at speed
        shooter.spinUpCommand(2800),
        // 2. Keep shooter running while feeding
        Commands.parallel(
            Commands.run(() -> shooter.setRPM(2800), shooter),
            feeder.feedCommand(),
            Commands.waitSeconds(0.125).andThen(floor.feedCommand())
        )
    ).finallyDo(() -> shooter.stop())
     .withTimeout(10)
     .withName("Simple Shoot Auton");
  }

  /* Expose subsystems if needed */
  public CommandSwerveDrivetrain getSwerve() {
    return swerve;
  }

  /** Returns the driver controller HID so Robot can set rumble. */
  public GenericHID getDriverController() {
    return driver.getHID();
  }

  /** Returns the aux controller HID so Robot can set rumble. */
  public GenericHID getAuxController() {
    return aux.getHID();
  }
}
