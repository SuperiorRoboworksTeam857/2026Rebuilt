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

  public Spindexer() {
    spindexerConfig.idleMode(IdleMode.kBrake);

    spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  public void runSpindexer(double speed) {
    spindexerMotor.set(speed * Constants.SpindexerConstants.spindexerSpeedMultiplier);
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("spindexer motor actual RPM", spindexerMotor.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
