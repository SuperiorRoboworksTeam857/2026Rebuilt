// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private TalonSRX intakePowerMotor = new TalonSRX(Constants.IntakeConstants.intakePowerMotor);
  private SparkFlex intakeExtensionMotor = new SparkFlex(Constants.IntakeConstants.intakeExtensionMotor,
      MotorType.kBrushless);
  private SparkFlexConfig intakeExtensionConfig = new SparkFlexConfig();

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
    runIntake(1);
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

  public void extendIntake() {
    double speed = 0;
    double currentPos = intakeExtensionMotor.getEncoder().getPosition();

    if (currentPos < Constants.IntakeConstants.intakeExtendedPosition) {
      speed = 0.1;
    } else {
      speed = 0;
    }
    intakeExtensionMotor.set(speed);
  }

  public void retractIntake() {
    double speed = 0;
    double currentPos = intakeExtensionMotor.getEncoder().getPosition();

    if (currentPos > Constants.IntakeConstants.intakeContractedPosition) {
      speed = -0.1;
    } else {
      speed = 0;
    }
    intakeExtensionMotor.set(speed);
  }

  public boolean isIntakeExtended() {
    return Math.abs(intakeExtensionMotor.getEncoder().getPosition() - Constants.IntakeConstants.intakeExtendedPosition) < 1;
  }

  public boolean isIntakeRetracted() {
    return Math.abs(intakeExtensionMotor.getEncoder().getPosition() - Constants.IntakeConstants.intakeContractedPosition) < 1;
  }
  // in case we want to manually see these things
  // comment out the default command at the beginning
  @Override
  public void periodic(){
    SmartDashboard.putNumber("intakeExtensionActual", intakeExtensionMotor.getEncoder().getPosition());
  }
}
