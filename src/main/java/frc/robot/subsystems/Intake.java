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
  // NOT doing this as there's no way to reset the reference position
  // instead using the ProfiledPIDController so we can manually put in the output
  // private SparkClosedLoopController intakeExtensionController =
  // intakeExtensionMotor.getClosedLoopController();
  private final TrapezoidProfile.Constraints intakeExtensionConstraints = new TrapezoidProfile.Constraints(20, 80);
  private final ProfiledPIDController intakeExtensionController = new ProfiledPIDController(
      Constants.IntakeConstants.intakeKP, Constants.IntakeConstants.intakeKI, Constants.IntakeConstants.intakeKD,
      intakeExtensionConstraints, Constants.deltaTime);
  private boolean shouldBeExtended = false;

  public Intake() {
    // define the configuration for the positionFactor and velocityFactor for the
    // intake extension motor
    // from 2025 Elevator code

    // SmartDashboard.putNumber("intakeExtensionInches",0.0);


    double sprocketDiameter = 2; // 22 teeth at 0.25 inch pitch
    double gearRatio = 5; // 20:1
    double driveConversionPositionFactor = (sprocketDiameter * Math.PI) / gearRatio;
    double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

    intakeExtensionConfig.encoder.positionConversionFactor(driveConversionVelocityFactor);
    intakeExtensionConfig.encoder.velocityConversionFactor(driveConversionVelocityFactor);
    intakeExtensionConfig.idleMode(IdleMode.kCoast); // coast mode to avoid holding the position

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
    intakeExtensionController.setGoal(setpoint);
    // set based on the goal
    intakeExtensionMotor.set(intakeExtensionController.calculate(intakeExtensionMotor.getEncoder().getPosition()));
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

  // // in case we want to manually see these things
  // // comment out the default command at the beginning
  // @Override
  // public void periodic(){
  //   double intakePosition = SmartDashboard.getNumber("intakeExtensionInches",0.0);

  //   intakeExtensionController.setGoal(intakePosition);
  //   // if it's 0, then don't set any power...
  //   if(intakePosition == 0){
  //     intakeExtensionMotor.set(0);
  //   }else{
  //     intakeExtensionMotor.set(intakeExtensionController.calculate(intakeExtensionMotor.getEncoder().getPosition()));
  //   }

  //   SmartDashboard.getNumber("intakeExtensionActual", intakeExtensionMotor.getEncoder().getPosition());
  // }
}
