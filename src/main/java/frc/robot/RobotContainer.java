// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAndDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Quest;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  /* Controllers */
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController aux    = new CommandXboxController(1);

  /* Subsystems */
  private final CommandSwerveDrivetrain swerve = TunerConstants.createDrivetrain();
  private final Quest quest = new Quest();
  private final Intake intake = new Intake();
  private final Floor floor = new Floor();
  private final Feeder feeder = new Feeder();
  private final Shooter shooter = new Shooter();
  private final Hood hood = new Hood();
  private final Hanger hanger = new Hanger();
  private final Limelight limelight = new Limelight("limelight");

  /* Auto chooser */
  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    // Feed QuestNav pose updates into the drivetrain's pose estimator
    swerve.setQuest(quest);

    // Register named commands BEFORE building the auto chooser.
    // These names must match the event marker / command names you set in PathPlanner GUI.
    registerNamedCommands();

    // Build the auto chooser from all autos in deploy/pathplanner/autos/
    autoChooser = AutoBuilder.buildAutoChooser();   // default auto will be Commands.none()
    SmartDashboard.putData("Auto Chooser", autoChooser);

    /* Set default commands */
    swerve.setDefaultCommand(
        SubsystemCommands.teleopDrive(
            swerve,
            () -> -driver.getLeftY(),  // forward/back (negate because stick Y is inverted)
            () -> -driver.getLeftX(),  // strafe left/right
            () -> -driver.getRightX() // rotation
        )

    );

    // Initialize Haptics
    //Haptics.initialize(driver, aux);

    configureBindings();
  }

  /**
   * Register named commands so PathPlanner event markers can trigger them.
   * Add any additional named commands here as you create more autos.
   */
  private void registerNamedCommands() {
  NamedCommands.registerCommand("intake",
    Commands.parallel(intake.intakeCommand(), floor.reverseIntakeAssistCommand()).withTimeout(4));
  NamedCommands.registerCommand("intake2",
    Commands.parallel(intake.intakeCommand(), floor.reverseIntakeAssistCommand()).withTimeout(3));
    NamedCommands.registerCommand("feed",
        Commands.parallel(feeder.feedCommand(), Commands.waitSeconds(0.25).andThen(floor.feedCommand())));
    NamedCommands.registerCommand("spinUpShooter", shooter.spinUpCommand(3000));
    NamedCommands.registerCommand("shoot",
        Commands.parallel(
            Commands.run(() -> shooter.setRPM(1450
            ), shooter),
            Commands.waitSeconds(3).andThen(feeder.feedCommand()),
            Commands.waitSeconds(3).andThen(floor.feedCommand()),
            Commands.waitSeconds(3).andThen(intake.slowHomeCommand())
        ).withTimeout(6).finallyDo(() -> shooter.stop()));
    NamedCommands.registerCommand("shootshort",
        Commands.parallel(
            Commands.run(() -> shooter.setRPM(1750), shooter),
            Commands.waitSeconds(0.5).andThen(feeder.feedCommand()),
            Commands.waitSeconds(0.5).andThen(floor.feedCommand())
        ).withTimeout(3).finallyDo(() -> shooter.stop()));
    NamedCommands.registerCommand("stopShooter", Commands.runOnce(() -> shooter.stop(), shooter));
    NamedCommands.registerCommand("intakeStop", Commands.runOnce(() -> intake.set(Intake.Speed.STOP), intake));
    
    // Quest-based path correction commands
    NamedCommands.registerCommand("resetPoseToQuest", subsystemCommands.resetPoseToQuest());
    NamedCommands.registerCommand("midPathQuestCorrection", subsystemCommands.midPathQuestCorrection());
    NamedCommands.registerCommand("questCorrectionGuard", subsystemCommands.questCorrectionGuard().withTimeout(15));
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
        quest,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );

  private void configureBindings() {
    /* ── Driver (port 0) – swerve only ── */

    /* B button → brake/lock wheels */
    driver.b().whileTrue(swerve.xStanceCommand());

    /* Back button → zero gyro heading */
    driver.back().onTrue(SubsystemCommands.zeroGyro(swerve));

    /* Start button → zero gyro heading (alternative) */
    driver.start().onTrue(SubsystemCommands.zeroGyro(swerve));

    /* X button → zero gyro heading */
    driver.x().onTrue(SubsystemCommands.zeroGyro(swerve));

  /* Y button → hold to auto-align to goal AprilTags while still driving manually */
  driver.y().whileTrue(
    new AimAndDriveCommand(
      swerve,
      limelight,
      () -> -driver.getLeftY(),
      () -> -driver.getLeftX()
    )
  );
   
    /* Right trigger → slow mode (30% speed) */
    driver.rightTrigger().whileTrue(
        SubsystemCommands.teleopDriveSlow(
            swerve,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()
        )
    );

    /* ── Aux (port 1) – mechanisms ── */
    aux.rightBumper().whileTrue(subsystemCommands.shootManually());
    aux.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
    aux.leftTrigger().whileTrue(Commands.parallel(intake.intakeCommand(), floor.reverseIntakeAssistCommand()));
    aux.leftBumper().onTrue(intake.runOnce(() -> intake.handleLeftBumperPress(true)));
    
    /* Y button → shoot with RPM based on distance from AprilTag */
    aux.y().whileTrue(subsystemCommands.ferry());
  }

  public Command getAutonomousCommand() {
    // Returns whichever auto is selected on the SmartDashboard/Shuffleboard chooser.
    // These autos are built from the .auto files in deploy/pathplanner/autos/
    return autoChooser.getSelected();
  }

  /* Expose subsystems if needed */
  public CommandSwerveDrivetrain getSwerve() {
    return swerve;
  }

/*   Trigger pulseTrigger = new Trigger(() -> GameState.shouldRumble())
    .onTrue(Commands.runOnce(() -> {
      Haptics.getInstance().pulse(3, 1, 1);
    })); */

  // TODO: Investigate need of methods below
  /** Returns the driver controller HID so Robot can set rumble. */
  public GenericHID getDriverController() {
    return driver.getHID();
  }

  /** Returns the aux controller HID so Robot can set rumble. */
  public GenericHID getAuxController() {
    return aux.getHID();
  }
}