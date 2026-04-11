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

public class Spindexer extends SubsystemBase {

  // one spindexerMotor in the middle that spins to bring the fuel cells around
  private SparkFlex spindexerMotor = new SparkFlex(Constants.SpindexerConstants.spindexerWheel, MotorType.kBrushless);
  private SparkFlexConfig spindexerConfig = new SparkFlexConfig(); // to handle the PID loop of the middle loop
  private double targetSpindexerSpeed = 0.4;

  private SparkFlex spindexerAgitator = new SparkFlex(Constants.SpindexerConstants.spindexerAgitator,
      MotorType.kBrushless);
  private SparkFlexConfig spindexerAgitatorConfig = new SparkFlexConfig(); // to handle the PID loop of the middle loop

  public Spindexer() {
    spindexerConfig.idleMode(IdleMode.kBrake);
    spindexerConfig.smartCurrentLimit(15, 15);
    spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    spindexerAgitatorConfig.idleMode(IdleMode.kCoast);
    spindexerAgitatorConfig.smartCurrentLimit(15, 15);
    spindexerAgitator.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runSpindexer(double speed) {
    spindexerMotor.set(speed * Constants.SpindexerConstants.spindexerSpeedMultiplier);
    spindexerAgitator.set(speed * Constants.SpindexerConstants.spindexerAgitatorMultiplier);
  }

  public void startSpindexer() {
    runSpindexer(targetSpindexerSpeed);
  }

  public void stopSpindexer() {
    runSpindexer(0);
  }

  public void reverseSpindexer() {
    runSpindexer(-targetSpindexerSpeed);
  }

  public void runSpindexerOnly(double speed) {
    spindexerMotor.set(speed * Constants.SpindexerConstants.spindexerSpeedMultiplier);
    spindexerAgitator.set(0);
  }
    public void startSpindexerOnly() {
    runSpindexerOnly(targetSpindexerSpeed);
  }
    public void reverseSpindexerOnly() {
    runSpindexerOnly(-targetSpindexerSpeed);
  }
  public void runAgitatorOnly(double speed) {
    spindexerMotor.set(0);
    spindexerAgitator.set(speed * Constants.SpindexerConstants.spindexerAgitatorMultiplier);
  }
    public void startAgitatorOnly() {
    runAgitatorOnly(targetSpindexerSpeed);
  }
    public void reverseAgitatorOnly() {
    runAgitatorOnly(-targetSpindexerSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("spindexer motor actual RPM", spindexerMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("agitator motor actual RPM", spindexerAgitator.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
