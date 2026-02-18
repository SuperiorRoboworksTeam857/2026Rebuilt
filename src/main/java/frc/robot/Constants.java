// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

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
  public static class Swerve {
    public static final double fastDriveSpeedMultiplier = 1.1; // NEEDS TO BE CHANGED
    public static final double normalDriveSpeedMultiplier = 0.7; // NEEDS TO BE CHANGED
    public static final double slowDriveSpeedMultiplier = 0.25; // NEEDS TO BE CHANGED

    public static final double stickDeadband = 0.1;
    public static final double robotMass = 50;
    public static final double momentOfInertia = 0;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-
    public static final double trackWidth = Units.inchesToMeters(18.75); // NEEDS TO BE CHANGED
    public static final double wheelBase = Units.inchesToMeters(18.75); // NEEDS TO BE CHANGED
    public static final double wheelDiameter = Units.inchesToMeters(4); // NEEDS TO BE CHANGED
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final double driveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // 6.75:1 // MIGHT NEED
    public static final double angleGearRatio = (150.0 / 7.0); // MIGHT NEED TO BE CHANGED
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;
    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 40;
    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;
    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;
    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second //4.5
    public static final double maxAngularVelocity = 6; // 11.5
    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 15;
      public static final int angleMotorID = 14;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(84.375);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 13;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(126.211);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 16;
      public static final int angleMotorID = 17;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(141.592);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(92.549 + 180);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  // constants for the Shooter
  // includes the 2 motors for the actual flywheel
  //   and one for the turret that changes the angle of the shooter
  public static final class ShooterConstants {
    public static final int shooterMotor1 = 22;
    public static final int shooterMotor2 = 23;
    public static final double shooterSpeedMultiplier = 1; // basic speed multiplier
    public static final int turretMotor = 21; 

    // PID constants for the spindexer wheel
    // TODO: mess with these values to get something right
    public static final double shooterKP = 0.1;
    public static final double shooterKI = 0;
    public static final double shooterKD = 0;

    // boolean to see if we should use the PID loop (for testing)
    public static final boolean usePID = false;
  }

  // constants for the Shooter
  // 2.9.26 unknown what this will look like
  public static final class IntakeConstants {
    public static final int intakeMotor = 0; // needs to be changed
  }

  // constants for the Spindexer
  // 2.9.26 unknown what this will look like
  public static final class SpindexerConstants {
    public static final int spindexerWheel = 24; // will spin in the center
    public static final double spindexerSpeedMultiplier = 1; // basic speed multiplier

    // PID constants for the spindexer wheel
    // TODO: mess with these values to get something right
    public static final double spindexerKP = 0.1;
    public static final double spindexerKI = 0;
    public static final double spindexerKD = 0;

    // boolean to see if we should use the PID loop (for testing)
    public static final boolean usePID = false;
  }
  // constants for the Feeder
  // 2.9.26 unknown what this will look like
  public static final class FeederConstants {
    public static final int feederWheelFront = 25;
    public static final int feederWheelBack = 26;
    public static final double feederSpeedMultiplier = 1; // basic speed multiplier

    // PID constants for the feeder wheel
    // TODO: mess with these values to get something right
    public static final double feederKP = 0.1;
    public static final double feederKI = 0;
    public static final double feederKD = 0;

    // boolean to see if we should use the PID loop (for testing)
    public static final boolean usePID = false;
  }

}