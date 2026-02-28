// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.auto.NamedCommands;

// import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /* Subsystems */
  private final Shooter m_Shooter = new Shooter();
  private final Swerve m_Swerve = new Swerve();
  private final Spindexer m_spindexer = new Spindexer();
  private final Intake m_intake = new Intake();

  private final Feeder m_feeder = new Feeder();

  /* Controllers */
  private final Joystick gamepad = new Joystick(0);
  private final Joystick driverStick = new Joystick(1);
  private final Joystick buttonBox = new Joystick(2);

  /* Drive Controls */
  private final int translationAxis = Joystick.AxisType.kY.value;
  private final int strafeAxis = Joystick.AxisType.kX.value;
  private final int rotationAxis = Joystick.AxisType.kZ.value;

  private final JoystickButton robotCentric = new JoystickButton(driverStick, 4);
  private final JoystickButton zeroGyro = new JoystickButton(driverStick, 3);

  private final JoystickButton slowSpeed = new JoystickButton(driverStick, 2);
  private final JoystickButton highSpeed = new JoystickButton(driverStick, 1);

  private final JoystickButton shootShooter = new JoystickButton(gamepad,
      Constants.ControllerConstants.shootShooterButton);
    private final JoystickButton manualShooter = new JoystickButton(gamepad,
      Constants.ControllerConstants.manualShooterButton);
  private final POVButton reverseSpindexer = new POVButton(gamepad, Constants.ControllerConstants.reverseSpindexer);
  private final POVButton forwardSpindexer = new POVButton(gamepad, Constants.ControllerConstants.forwardSpindexer);
  private final POVButton reverseFeeder = new POVButton(gamepad, Constants.ControllerConstants.reverseFeeder);
  private final POVButton forwardFeeder = new POVButton(gamepad, Constants.ControllerConstants.forwardFeeder);
  private final JoystickButton intakeIn = new JoystickButton(gamepad, Constants.ControllerConstants.intakeInButton);
  private final JoystickButton intakeOut = new JoystickButton(gamepad, Constants.ControllerConstants.intakeOutButton);
  private final Trigger alignAndShoot = new Trigger(
      () -> gamepad.getRawAxis(Constants.ControllerConstants.alignAndShoot) > 0.7);
  private final Trigger intakeAndSpindex = new Trigger(
      () -> gamepad.getRawAxis(Constants.ControllerConstants.intakeAndSpindex) > 0.7);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_Swerve.setDefaultCommand(
        new TeleopSwerve(
            m_Swerve,
            () -> -driverStick.getRawAxis(translationAxis),
            () -> driverStick.getRawAxis(strafeAxis),
            () -> driverStick.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean(),
            () -> slowSpeed.getAsBoolean(),
            () -> highSpeed.getAsBoolean()));

    NamedCommands.registerCommand("startSpindexer",
        new InstantCommand(
            () -> m_spindexer.startSpindexer(), m_spindexer));
    NamedCommands.registerCommand("startIntake",
        new InstantCommand(
            () -> m_intake.startIntake(), m_intake));
    NamedCommands.registerCommand("startShooterForFiveSeconds",
        new SequentialCommandGroup(
            new RunCommand(
                () -> m_Shooter.runShooterThenRest(m_feeder, m_spindexer), m_Shooter).withTimeout(5)));

    NamedCommands.registerCommand("stopSpindexer",
        new InstantCommand(
            () -> m_spindexer.stopSpindexer(), m_spindexer));
    NamedCommands.registerCommand("stopIntake",
        new InstantCommand(
            () -> m_intake.stopIntake(), m_intake));
    NamedCommands.registerCommand("stopEverything",
        new SequentialCommandGroup(
            new InstantCommand(
                () -> m_spindexer.stopSpindexer(), m_spindexer),
            new InstantCommand(
                () -> m_intake.stopIntake(), m_intake),
            new InstantCommand(
                () -> m_feeder.stopFeeder(), m_feeder),
            new InstantCommand(
                () -> m_Shooter.stopShooter(), m_Shooter)));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // alignAndShoot.whileTrue(
    // new SequentialCommandGroup(
    // new InstantCommand(
    // () -> m_Shooter.startShooter(), m_Shooter),
    // new WaitUntilCommand(m_Shooter::isShooterAtSpeed),
    // new InstantCommand(
    // () -> m_feeder.startFeeder(), m_feeder),
    // new InstantCommand(
    // () -> m_spindexer.startSpindexer(), m_spindexer))

    // );
    shootShooter.whileTrue(
        new RunCommand(
            () -> m_Shooter.runShooterThenRest(m_feeder, m_spindexer), m_Shooter));
    intakeAndSpindex.whileTrue(
        new SequentialCommandGroup(
            new InstantCommand(
                () -> m_intake.startIntake(), m_intake),
            new InstantCommand(
                () -> m_spindexer.startSpindexer(), m_spindexer)));
    intakeAndSpindex.onFalse(
        new SequentialCommandGroup(
            new InstantCommand(
                () -> m_intake.stopIntake(), m_intake),
            new InstantCommand(
                () -> m_spindexer.stopSpindexer(), m_spindexer)));

    alignAndShoot.onFalse(
        new SequentialCommandGroup(
            new InstantCommand(
                () -> m_Shooter.stopShooter(), m_Shooter),
            new InstantCommand(
                () -> m_feeder.stopFeeder(), m_feeder),
            new InstantCommand(
                () -> m_spindexer.stopSpindexer(), m_spindexer))

    );

    manualShooter.whileTrue(
      new InstantCommand(
        () -> m_Shooter.startShooter(), m_Shooter
      )
    );

    manualShooter.onFalse(
      new InstantCommand(
        () -> m_Shooter.stopShooter(), m_Shooter
      )
    );

    reverseSpindexer.whileTrue(
        new InstantCommand(
            () -> m_spindexer.reverseSpindexer(), m_spindexer).onlyIf(
                () -> !forwardSpindexer.getAsBoolean()));
    forwardSpindexer.whileTrue(
        new InstantCommand(
            () -> m_spindexer.startSpindexer(), m_spindexer));
    reverseFeeder.whileTrue(
        new InstantCommand(
            () -> m_feeder.reverseFeeder(), m_feeder).onlyIf(
                () -> !forwardFeeder.getAsBoolean()));
    forwardFeeder.whileTrue(
        new InstantCommand(
            () -> m_feeder.startFeeder(), m_feeder));
    intakeIn.whileTrue(
        new InstantCommand(
            () -> m_intake.startIntake(), m_intake));
    intakeOut.whileTrue(
        new InstantCommand(
            () -> m_intake.reverseIntake(), m_intake).onlyIf(
                () -> (!intakeIn.getAsBoolean() && !intakeAndSpindex.getAsBoolean())));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.blankAuto();
  }
}
