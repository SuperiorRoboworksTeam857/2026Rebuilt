// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.LimelightRead;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    private final double FULL_SPEED = 1;
    private final double NORMAL_SPEED = 0.7;
    private final double SLOW_SPEED = 0.3;

    /* Controllers */
    private final Joystick gamepad = new Joystick(0);
    private final Joystick driverStick = new Joystick(1);
    private final Joystick buttonBox = new Joystick(2);

    /* Subsystems */
    private final SwerveSubsystem m_swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neoVortex"));
    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerve.getSwerveDrive(),
            () -> driverStick.getY() * -1,
            () -> driverStick.getX() * -1)
            .withControllerRotationAxis(() -> driverStick.getZ() * -1)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(NORMAL_SPEED)
            .allianceRelativeControl(true);
    SwerveInputStream driveAngularVelocitySlow = driveAngularVelocity.copy().scaleTranslation(SLOW_SPEED)
            .scaleRotation(SLOW_SPEED);
    SwerveInputStream driveAngularVelocityFast = driveAngularVelocity.copy().scaleTranslation(FULL_SPEED)
            .scaleRotation(FULL_SPEED);

    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

    private final Intake m_intake = new Intake();
    private final Spindexer m_spindexer = new Spindexer();
    private final Feeder m_feeder = new Feeder();
    private final Shooter m_shooter = new Shooter(m_swerve);
    private final Limelight s_Limelight = new Limelight();

    private final JoystickButton highSpeed = new JoystickButton(driverStick, 1);
    private final JoystickButton slowSpeed = new JoystickButton(driverStick, 2);
    private final JoystickButton zeroGyro = new JoystickButton(driverStick, 3);
    private final JoystickButton robotCentric = new JoystickButton(driverStick, 4);
    private final JoystickButton centerModules = new JoystickButton(driverStick, 5);
    private final JoystickButton xLockWheels = new JoystickButton(driverStick, 6);

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
    private final JoystickButton intakeExtend = new JoystickButton(gamepad,
            Constants.ControllerConstants.intakeExtendButton);
    private final JoystickButton intakeContract = new JoystickButton(gamepad,
            Constants.ControllerConstants.intakeContractButton);

    private final Trigger alignAndShoot = new Trigger(
            () -> gamepad.getRawAxis(Constants.ControllerConstants.alignAndShoot) > 0.7);
    private final Trigger intakeAndSpindex = new Trigger(
            () -> gamepad.getRawAxis(Constants.ControllerConstants.intakeAndSpindex) > 0.7);

    public final AprilTagFieldLayout layout;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // // Start camera streams for both webcams
        // CameraServer.startAutomaticCapture();
        // CameraServer.startAutomaticCapture();

        s_Limelight.turnOnDriverCam();
        s_Limelight.enableLimelight(false);
        s_Limelight.setPipeline(Limelight.Pipeline.AprilTags);

        NamedCommands.registerCommand("startSpindexer",
                new InstantCommand(
                        () -> m_spindexer.startSpindexer(), m_spindexer));
        NamedCommands.registerCommand("startIntake",
                new InstantCommand(
                        () -> m_intake.startIntake(), m_intake));
        NamedCommands.registerCommand("startShooterForFiveSeconds",
                new SequentialCommandGroup(
                        new RunCommand(
                                () -> m_shooter.runShooterThenRest(m_feeder, m_spindexer), m_shooter).withTimeout(5)));

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
                                () -> m_shooter.stopShooter(), m_shooter)));

        Command driveFieldOrientedAnglularVelocity = m_swerve.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity = m_swerve.driveFieldOriented(driveRobotOriented); // TODO: add
                                                                                                     // control to flip
                                                                                                     // to robot centric
        m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        m_intake.setDefaultCommand(new RunCommand(() -> m_intake.stopIntakeAndExtension(), m_intake));
        m_spindexer.setDefaultCommand(new RunCommand(() -> m_spindexer.stopSpindexer(), m_spindexer));
        m_feeder.setDefaultCommand(new RunCommand(() -> m_feeder.stopFeeder(), m_feeder));
        m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.stopShooter(), m_shooter));

        s_Limelight.setDefaultCommand(new LimelightRead(s_Limelight));

        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {
        // SWERVE CONTROLS
        zeroGyro.onTrue(new InstantCommand(() -> m_swerve.zeroGyroWithAlliance()));
        centerModules.whileTrue(m_swerve.centerModulesCommand());
        xLockWheels.whileTrue(new RunCommand(() -> m_swerve.lock(), m_swerve));

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
        shootShooter.whileTrue(new RunCommand(() -> m_shooter.runShooterThenRest(m_feeder, m_spindexer), m_shooter));

        // shootShooter.onTrue(
        // new SequentialCommandGroup(
        // new ParallelRaceGroup(
        // new RunCommand(
        // () -> m_shooter.startShooter(), m_shooter
        // ),
        // new WaitCommand(.7)
        // ),
        // new ParallelRaceGroup(
        // new RunCommand(
        // () -> m_shooter.startShooter(), m_shooter
        // ),
        // new RunCommand(
        // () -> m_feeder.startFeeder(), m_feeder
        // ),
        // new RunCommand(
        // () -> m_spindexer.startSpindexer(), m_spindexer
        // )
        // )
        // )
        // );

        // 1. extend intake if not already
        // 2. once extended, begin the intake
        // 3. run the spindexer as well
        // intakeAndSpindex.whileTrue(
        // new ParallelRaceGroup(
        // new RunCommand(
        // () -> m_intake.startIntake(), m_intake),
        // new RunCommand(
        // () -> m_spindexer.startSpindexer(), m_spindexer)));

        intakeAndSpindex.whileTrue(
                new SequentialCommandGroup(
                        new InstantCommand(
                                () -> m_intake.setIntakeExtension(true), m_intake),
                        new RunCommand(
                                () -> m_intake.enforceIntakeExtension(), m_intake)
                                .until(m_intake::isIntakeAtTargetExtension),
                        new ParallelRaceGroup(
                                new RunCommand(
                                        () -> m_intake.startIntake(), m_intake),
                                new RunCommand(
                                        () -> m_spindexer.startSpindexer(), m_spindexer))));

        manualShooter.whileTrue(
                new RunCommand(
                        () -> m_shooter.startShooter(), m_shooter));

        reverseSpindexer.whileTrue(
                new RunCommand(
                        () -> m_spindexer.reverseSpindexer(), m_spindexer).onlyIf(
                                () -> !forwardSpindexer.getAsBoolean()));

        forwardSpindexer.whileTrue(
                new RunCommand(
                        () -> m_spindexer.startSpindexer(), m_spindexer));

        reverseFeeder.whileTrue(
                new RunCommand(
                        () -> m_feeder.reverseFeeder(), m_feeder).onlyIf(
                                () -> !forwardFeeder.getAsBoolean()));

        forwardFeeder.whileTrue(
                new RunCommand(
                        () -> m_feeder.startFeeder(), m_feeder));

        intakeIn.whileTrue(new RunCommand(() -> m_intake.startIntake(), m_intake));
        intakeOut.whileTrue(
                new RunCommand(
                        () -> m_intake.reverseIntake(), m_intake).onlyIf(
                                () -> (!intakeIn.getAsBoolean() && !intakeAndSpindex.getAsBoolean())));

        intakeExtend.whileTrue(
                new SequentialCommandGroup(
                        new InstantCommand(
                                () -> m_intake.setIntakeExtension(true), m_intake),
                        new RunCommand(
                                () -> m_intake.enforceIntakeExtension(), m_intake)));

        intakeContract.whileTrue(
                new SequentialCommandGroup(
                        new InstantCommand(
                                () -> m_intake.setIntakeExtension(true), m_intake),
                        new RunCommand(
                                () -> m_intake.enforceIntakeExtension(), m_intake))
                        .onlyIf(
                                () -> !intakeExtend.getAsBoolean()));

    }

    public void setMotorBrake(boolean brake) {
        m_swerve.setMotorBrake(brake);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        if (buttonBox.getRawButton(3)) {
            return new PathPlannerAuto("test auto");
        } else if (buttonBox.getRawButton(4)) {
            return new PathPlannerAuto("Left Trench Auto");
        }

        // An example command will be run in autonomous
        return Autos.blankAuto();
    }
}
