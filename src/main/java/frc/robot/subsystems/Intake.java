// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private TalonSRX intakePowerMotor = new TalonSRX(Constants.IntakeConstants.intakePowerMotor);
  private SparkFlex intakeExtensionMotor = new SparkFlex(Constants.IntakeConstants.intakeExtensionMotor,
      MotorType.kBrushless);
  private SparkFlexConfig intakeExtensionConfig = new SparkFlexConfig();

  private boolean shouldBeExtended = false;

  public Intake() {
    double sprocketDiameter = 2; // inches
    double gearRatio = 5; // 5:1
    double driveConversionPositionFactor = (sprocketDiameter * Math.PI) / gearRatio;
    double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

    intakeExtensionConfig.encoder.positionConversionFactor(driveConversionPositionFactor);
    intakeExtensionConfig.encoder.velocityConversionFactor(driveConversionVelocityFactor);
    intakeExtensionConfig.idleMode(IdleMode.kCoast); // coast mode to avoid holding the position

    intakeExtensionConfig.inverted(true);

    intakeExtensionMotor.configure(intakeExtensionConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void runIntake(double speed) {
    intakePowerMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void startIntake() {
    runIntake(0.75);
  }

  public void stopIntake() {
    runIntake(0);
  }

  public void reverseIntake() {
    runIntake(-0.5);
  }

  public void stopIntakeAndExtension() {
    runIntake(0);
    intakeExtensionMotor.set(0);
  }

  public void stopIntakeExtension() {
    intakeExtensionMotor.set(0); // set to 0 power so coast happens
  }

  public void enforceIntakeExtension() {
    double setpoint = shouldBeExtended ? Constants.IntakeConstants.intakeExtendedPosition
        : Constants.IntakeConstants.intakeContractedPosition;

    double currentPos = intakeExtensionMotor.getEncoder().getPosition();

    double speed = 0;
    if (setpoint > currentPos) {
      speed = 0.05;
    } else if (setpoint < currentPos) {
      speed = -0.05;
    }

    intakeExtensionMotor.set(clamp(speed, -0.05, 0.05));
  }

  public void setIntakeExtension(boolean extended) {
    shouldBeExtended = extended;
  }

  public boolean isIntakeAtTargetExtension() {
    double targetPosition = shouldBeExtended ? Constants.IntakeConstants.intakeExtendedPosition
        : Constants.IntakeConstants.intakeContractedPosition;

    // change to fit the allowable position
    // within half an inch
    return Math.abs(intakeExtensionMotor.getEncoder().getPosition() - targetPosition) < .5;
  }

  // in case we want to manually see these things
  // comment out the default command at the beginning
  @Override
  public void periodic(){
    SmartDashboard.putNumber("intakeExtensionActual", intakeExtensionMotor.getEncoder().getPosition());
  }

  public static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}
