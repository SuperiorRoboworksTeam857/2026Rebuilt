// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {

  // one spindexerMotor in the middle that spins to bring the fuel cells around
  private SparkFlex spindexerMotor = new SparkFlex(Constants.SpindexerConstants.spindexerWheel, MotorType.kBrushless);
  private SparkFlexConfig spindexerConfig = new SparkFlexConfig(); // to handle the PID loop of the middle loop
  private SparkClosedLoopController spindexerController = spindexerMotor.getClosedLoopController();
  private double targetSpindexerSpeed = 0;

  /** Creates a new ExampleSubsystem. */
  public Spindexer() {
    SmartDashboard.putNumber("spindexerMotorSpeed", 0);

    // setup PID parameters
    spindexerConfig.closedLoop
      .p(Constants.SpindexerConstants.spindexerKP)
      .i(Constants.SpindexerConstants.spindexerKI)
      .d(Constants.SpindexerConstants.spindexerKD);

    spindexerConfig.idleMode(IdleMode.kBrake);

    spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSpindexerVelocity(double velocity){
    spindexerController.setSetpoint(velocity, ControlType.kVelocity);
  }
  
  public void runSpindexer(double speed) {
    spindexerMotor.set(speed * Constants.SpindexerConstants.spindexerSpeedMultiplier);
  }
  public void powerSpindexer(double speed) {
    if(Constants.SpindexerConstants.usePID)
      setSpindexerVelocity(speed);
    else
      runSpindexer(speed);
  }
  public void startSpindexer() {
    powerSpindexer(targetSpindexerSpeed);

  }
  public void stopSpindexer() {
    powerSpindexer(0);
  }
  public void reverseSpindexer() {
    powerSpindexer(-targetSpindexerSpeed);
  }

  @Override
  public void periodic() {
    // grab from the dashboard the speed for the spindexer and set it
    // default 0 so it doesn't run when we don't want it to
    double spindexerMotorPower = SmartDashboard.getNumber("spindexerMotorSpeed", 0f);
    targetSpindexerSpeed = spindexerMotorPower;
    // choose between PID control and just setting the power to the motor based on the constant
    // if(Constants.SpindexerConstants.usePID)
    //   setSpindexerVelocity(spindexerMotorPower);
    // else
    //   runSpindexer(spindexerMotorPower);

    SmartDashboard.putNumber("spindexer motor actual RPM", spindexerMotor.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
