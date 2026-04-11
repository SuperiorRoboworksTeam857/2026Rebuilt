// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

  // one feederMotor in the middle that spins to bring the fuel cells around
  private SparkFlex feederMotorFront = new SparkFlex(Constants.FeederConstants.feederWheelFront, MotorType.kBrushless);
  private SparkFlex feederMotorBack = new SparkFlex(Constants.FeederConstants.feederWheelBack, MotorType.kBrushless);
  private SparkFlexConfig feederConfigFront = new SparkFlexConfig(); // to handle the PID loop of the middle loop
  private SparkFlexConfig feederConfigBack = new SparkFlexConfig(); // to handle the PID loop of the middle loop
  private double targetFeederSpeed = -0.4;

  public Feeder() {
    feederConfigBack.inverted(true);

    feederConfigFront.idleMode(IdleMode.kBrake);
    feederConfigBack.idleMode(IdleMode.kBrake);

    feederMotorFront.configure(feederConfigFront, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feederMotorBack.configure(feederConfigBack, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runFeeder(double speed) {
    feederMotorFront.set(speed * Constants.FeederConstants.feederSpeedMultiplier * 3.0/2.25);
    feederMotorBack.set(speed * Constants.FeederConstants.feederSpeedMultiplier);
  }

  public void startFeeder() {
    runFeeder(targetFeederSpeed);
  }
  public void stopFeeder() {
    runFeeder(0);
  }
  public void reverseFeeder() {
    runFeeder(-targetFeederSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("feeder motor front actual RPM", feederMotorFront.getEncoder().getVelocity());
    SmartDashboard.putNumber("feeder motor back actual RPM", feederMotorBack.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
