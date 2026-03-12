// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double deltaTime = 0.02;

  public static class Swerve {
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(10);
  }

 public static final class AutonConstants {
   public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
   public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
 }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds%
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }


  // constants for the Shooter
  // includes the 2 motors for the actual flywheel
  // and one for the turret that changes the angle of the shooter
  public static final class ShooterConstants {
    public static final int shooterMotor1 = 22;
    public static final int shooterMotor2 = 23;
    public static final double shooterSpeedMultiplier = 1; // basic speed multiplier
    public static final int turretMotor = 21;

    // PID constants for the spindexer wheel
    // TODO: mess with these values to get something right
    public static final double shooterKP = 0;
    public static final double shooterKI = 0;
    public static final double shooterKD = 0;
    public static final double shooterKV = 0.0018;

    // Turret constants for the turret
    public static final double turretKP = 2.5;
    public static final double turretKI = 0;
    public static final double turretKD = 0;

    // Convert turret motor angle to be the actual turret angle in rotations
    public static final double turretPositionFactor = (20.0 / 200.0) * (1.0 / 5.0); // 1/50
    public static final double turretMinLimit = -0.45; // -0.45
    public static final double turretMaxLimit = 0.4; // 0.4


    public static final Pose2d redHubLocation =  new Pose2d(Units.inchesToMeters(469.11),
                                                            Units.inchesToMeters(158.845),
                                                            Rotation2d.kZero);
    public static final Pose2d blueHubLocation = new Pose2d(Units.inchesToMeters(182.11),
                                                            Units.inchesToMeters(158.845),
                                                            Rotation2d.kZero);

    public static final Pose2d blueDownCorner =  new Pose2d(Units.inchesToMeters(12),    // wall at 0
                                                            Units.inchesToMeters(25.37), // wall at 0
                                                            Rotation2d.kZero);
    public static final Pose2d blueUpCorner =    new Pose2d(Units.inchesToMeters(12),    // wall at 0
                                                            Units.inchesToMeters(292.31),// wall at 318
                                                            Rotation2d.kZero);

    public static final Pose2d redDownCorner =  new Pose2d(Units.inchesToMeters(638),    // wall at 650
                                                           Units.inchesToMeters(25.37),  // wall at 0
                                                           Rotation2d.kZero);
    public static final Pose2d redUpCorner =    new Pose2d(Units.inchesToMeters(638),    // wall at 650
                                                           Units.inchesToMeters(292.31), // wall at 318
                                                           Rotation2d.kZero);
  }

  // constants for the Spindexer
  // 2.9.26 unknown what this will look like
  public static final class SpindexerConstants {
    public static final int spindexerWheel = 24; // will spin in the center
    public static final double spindexerSpeedMultiplier = 1; // basic speed multiplier
  }

  // constants for the Feeder
  // 2.9.26 unknown what this will look like
  public static final class FeederConstants {
    public static final int feederWheelFront = 25;
    public static final int feederWheelBack = 26;
    public static final double feederSpeedMultiplier = 1; // basic speed multiplier
  }

  public static final class ControllerConstants {
    public static final int shootShooterButton = XboxController.Button.kY.value;
    public static final int manualShooterButton = XboxController.Button.kX.value;
    public static final int intakeInButton = XboxController.Button.kRightBumper.value;
    public static final int intakeOutButton = XboxController.Button.kLeftBumper.value;
    public static final int alignAndShoot = XboxController.Axis.kRightTrigger.value;
    public static final int intakeAndSpindex = XboxController.Axis.kLeftTrigger.value;
    public static final int intakeExtendButton = XboxController.Button.kA.value;
    public static final int intakeContractButton = XboxController.Button.kB.value;
    public static final int reverseSpindexer = 270;
    public static final int forwardSpindexer = 90;
    public static final int reverseFeeder = 180;
    public static final int forwardFeeder = 0;
  }

  public static final class IntakeConstants {
    public static final int intakePowerMotor = 28;
    public static final int intakeExtensionMotor = 27;
    public static final double intakeSpeedMultiplier = 1; // basic speed multiplier

    public static final double intakeExtendedPosition = 11;
    public static final double intakeContractedPosition = 0;

    // PID constants for the extension
    public static final double intakeKP = 0.1;
    public static final double intakeKI = 0;
    public static final double intakeKD = 0;
  }

}