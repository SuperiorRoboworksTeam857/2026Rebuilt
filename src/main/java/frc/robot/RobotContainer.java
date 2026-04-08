// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.LimelightRead;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeExtension;
import frc.robot.subsystems.LED;
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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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
    private final double SLOW_SPEED = 0.5;

    private double driveSpeedScaling = NORMAL_SPEED;
    private boolean robotCentricDriving = false;


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
            () -> driverStick.getY() * -1 * driveSpeedScaling,
            () -> driverStick.getX() * -1 * driveSpeedScaling)
            .withControllerRotationAxis(() -> driverStick.getZ() * -1 * driveSpeedScaling)
            .deadband(OperatorConstants.DEADBAND)
            .allianceRelativeControl(() -> !robotCentricDriving)
            .robotRelative(() -> robotCentricDriving);

    private final Intake m_intake = new Intake();
    private final IntakeExtension m_intakeExtension = new IntakeExtension();
    private final Spindexer m_spindexer = new Spindexer();
    private final Feeder m_feeder = new Feeder();
    private final Shooter m_shooter = new Shooter(m_swerve);
    private final Limelight s_Limelight = new Limelight();
    private final LED s_LED = new LED();

    private final JoystickButton highSpeed = new JoystickButton(driverStick, 1);
    private final JoystickButton slowSpeed = new JoystickButton(driverStick, 2);
    private final JoystickButton zeroGyro = new JoystickButton(driverStick, 3);
    private final JoystickButton robotCentric = new JoystickButton(driverStick, 4);
    private final JoystickButton centerModules = new JoystickButton(driverStick, 5);
    private final JoystickButton xLockWheels = new JoystickButton(driverStick, 6);
    private final JoystickButton alignToField = new JoystickButton(driverStick, 7);

    private final JoystickButton shootShooter = new JoystickButton(gamepad,
            Constants.ControllerConstants.shootShooterButton);
    private final JoystickButton manualShooter = new JoystickButton(gamepad,
            Constants.ControllerConstants.manualShooterButton);

    private final JoystickButton reverseEverything = new JoystickButton(gamepad, Constants.ControllerConstants.reverseEverything);

    private final JoystickButton intakeIn = new JoystickButton(gamepad, Constants.ControllerConstants.intakeInButton);
    private final JoystickButton intakeOut = new JoystickButton(gamepad, Constants.ControllerConstants.intakeOutButton);
    private final POVButton intakeExtend = new POVButton(gamepad, Constants.ControllerConstants.intakeExtendPOV);
    private final POVButton intakeContract = new POVButton(gamepad, Constants.ControllerConstants.intakeRetractPOV);
    private final POVButton shooterUp = new POVButton(gamepad, Constants.ControllerConstants.shooterAdjustUp);
    private final POVButton shooterDown = new POVButton(gamepad, Constants.ControllerConstants.shooterAdjustDown);

    private final JoystickButton agitateIntake = new JoystickButton(gamepad, Constants.ControllerConstants.agitateIntakeButton);


    public final AprilTagFieldLayout layout;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // Start camera stream for webcam
        CameraServer.startAutomaticCapture();

        s_Limelight.turnOnDriverCam();
        s_Limelight.enableLimelight(false);
        s_Limelight.setPipeline(Limelight.Pipeline.AprilTags);



        NamedCommands.registerCommand("driveForwardOneSecond", m_swerve.driveForward().withTimeout(1));

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
        NamedCommands.registerCommand("startShooterForTenSeconds",
                new SequentialCommandGroup(
                        new RunCommand(
                                () -> m_shooter.runShooterThenRest(m_feeder, m_spindexer), m_shooter).withTimeout(10)));
        NamedCommands.registerCommand("startShooterForFifteenSeconds",
                new SequentialCommandGroup(
                        new RunCommand(
                                () -> m_shooter.runShooterThenRest(m_feeder, m_spindexer), m_shooter).withTimeout(15)));


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
        Command extendIntake =
                new RunCommand(() -> m_intakeExtension.extendIntake(), m_intakeExtension).withTimeout(1)
                        .until(m_intakeExtension::isIntakeExtended)
                        .andThen(new InstantCommand(() -> m_intakeExtension.stopIntakeExtension(), m_intakeExtension));
        Command retractIntake =
                 new RunCommand(() -> m_intakeExtension.retractIntake(), m_intakeExtension).withTimeout(1)
                        .until(m_intakeExtension::isIntakeRetracted)
                        .andThen(new InstantCommand(() -> m_intakeExtension.stopIntakeExtension(), m_intakeExtension));

        NamedCommands.registerCommand("extendIntake", extendIntake);
        NamedCommands.registerCommand("retractIntake", retractIntake);
        NamedCommands.registerCommand("agitateIntake",
                new RepeatCommand(new SequentialCommandGroup(retractIntake, extendIntake)));
                

        Command driveFieldOrientedAnglularVelocity = m_swerve.driveFieldOriented(driveAngularVelocity);

        m_swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        m_intake.setDefaultCommand(new RunCommand(() -> m_intake.stopIntake(), m_intake));
        m_intakeExtension.setDefaultCommand(new RunCommand(() -> m_intakeExtension.stopIntakeExtension(), m_intakeExtension));
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

        slowSpeed.whileTrue(new InstantCommand(() -> driveSpeedScaling = SLOW_SPEED))
                 .onFalse(new InstantCommand(() -> driveSpeedScaling = NORMAL_SPEED));
        highSpeed.whileTrue(new InstantCommand(() -> driveSpeedScaling = FULL_SPEED))
                 .onFalse(new InstantCommand(() -> driveSpeedScaling = NORMAL_SPEED));

        robotCentric.whileTrue(new InstantCommand(() -> robotCentricDriving = true))
                    .onFalse(new InstantCommand(() -> robotCentricDriving = false));

        // SHOOTER CONTROLS
        shootShooter.whileTrue(new RunCommand(() -> m_shooter.runShooterThenRest(m_feeder, m_spindexer), m_shooter));
        manualShooter.whileTrue(new RunCommand(() -> m_shooter.startShooter(), m_shooter));

        shooterUp.onTrue(new InstantCommand(() -> m_shooter.increaseManualAdjustment()));
        shooterDown.onTrue(new InstantCommand(() -> m_shooter.decreaseManualAdjustment()));
        


        // SPINDEXER AND FEEDER CONTROLS
        reverseEverything.whileTrue(
                new ParallelCommandGroup(
                        new RunCommand(() -> m_spindexer.reverseSpindexerOnly(), m_spindexer),
                        new RunCommand(() -> m_feeder.reverseFeeder(), m_feeder)
                )
        );
        
        // INTAKE CONTROLS
        intakeIn.whileTrue(new RunCommand(() -> m_intake.startIntake(), m_intake));
        intakeOut.whileTrue(new RunCommand(() -> m_intake.reverseIntake(), m_intake));

        intakeExtend.whileTrue(new RunCommand(() -> m_intakeExtension.extendIntake(), m_intakeExtension));
        intakeContract.whileTrue(new RunCommand(() -> m_intakeExtension.retractIntake(), m_intakeExtension));

        Command extendIntake =
                new RunCommand(() -> m_intakeExtension.extendIntake(), m_intakeExtension).withTimeout(0.8)
                        .until(m_intakeExtension::isIntakeExtended)
                        .andThen(new InstantCommand(() -> m_intakeExtension.stopIntakeExtension(), m_intakeExtension));
        Command retractIntake =
                 new RunCommand(() -> m_intakeExtension.retractIntake(), m_intakeExtension).withTimeout(0.8)
                        .until(m_intakeExtension::isIntakeRetracted)
                        .andThen(new InstantCommand(() -> m_intakeExtension.stopIntakeExtension(), m_intakeExtension));

        agitateIntake.whileTrue(
                new SequentialCommandGroup(
                        new InstantCommand(() -> m_intake.startIntake(), m_intake),
                        new RepeatCommand(new SequentialCommandGroup(retractIntake, extendIntake))
                )
        );

        // align to the field
        // aims to the nearest right angle
        alignToField.whileTrue(m_swerve.aimToNearestRight());
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
            return new PathPlannerAuto("Left Trench Auto");
        } else if (buttonBox.getRawButton(4)) {
            return new PathPlannerAuto("Left Trench Auto + Depot");
        } else if (buttonBox.getRawButton(5)) {
            return new PathPlannerAuto("Center Auto");
        } else if (buttonBox.getRawButton(6)) {
            return new PathPlannerAuto("Center Auto + Depot");
        } else if (buttonBox.getRawButton(7)) {
            return new PathPlannerAuto("Right Trench Auto");
        }

        // An example command will be run in autonomous
        return Autos.blankAuto();
    }
}
