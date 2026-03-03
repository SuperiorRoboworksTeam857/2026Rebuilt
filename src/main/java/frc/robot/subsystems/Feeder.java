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

public class Feeder extends SubsystemBase {

  // one feederMotor in the middle that spins to bring the fuel cells around
  private SparkFlex feederMotorFront = new SparkFlex(Constants.FeederConstants.feederWheelFront, MotorType.kBrushless);
  private SparkFlex feederMotorBack = new SparkFlex(Constants.FeederConstants.feederWheelBack, MotorType.kBrushless);
  private SparkFlexConfig feederConfigFront = new SparkFlexConfig(); // to handle the PID loop of the middle loop
  private SparkFlexConfig feederConfigBack = new SparkFlexConfig(); // to handle the PID loop of the middle loop
  private SparkClosedLoopController feederControllerFront = feederMotorFront.getClosedLoopController();
  private SparkClosedLoopController feederControllerBack = feederMotorBack.getClosedLoopController();
  private double targetFeederSpeed = 0;

  public Feeder() {
    SmartDashboard.putNumber("feederMotorSpeed", 0);

    // setup PID parameters
    feederConfigFront.closedLoop
      .p(Constants.FeederConstants.feederKP)
      .i(Constants.FeederConstants.feederKI)
      .d(Constants.FeederConstants.feederKD);

    feederConfigBack.closedLoop
      .p(Constants.FeederConstants.feederKP)
      .i(Constants.FeederConstants.feederKI)
      .d(Constants.FeederConstants.feederKD);

    feederConfigBack.inverted(true);

    feederConfigFront.idleMode(IdleMode.kBrake);
    feederConfigBack.idleMode(IdleMode.kBrake);

    feederMotorFront.configure(feederConfigFront, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feederMotorBack.configure(feederConfigBack, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setFeederVelocity(double velocity){
    feederControllerFront.setSetpoint(velocity, ControlType.kVelocity);
    feederControllerBack.setSetpoint(velocity, ControlType.kVelocity);
  }
  
  public void runFeeder(double speed) {
    feederMotorFront.set(speed * Constants.FeederConstants.feederSpeedMultiplier);
    feederMotorBack.set(speed * Constants.FeederConstants.feederSpeedMultiplier);
  }

  public void powerFeeder(double speed) {
    if(Constants.FeederConstants.usePID){
      setFeederVelocity(speed);
    } else {
      runFeeder(speed);
    }
  }
  public void startFeeder() {
    powerFeeder(targetFeederSpeed);
  }
  public void stopFeeder() {
    powerFeeder(0);
  }
  public void reverseFeeder() {
    powerFeeder(-targetFeederSpeed);
  }

  @Override
  public void periodic() {
    // grab from the dashboard the speed for the feeder and set it
    // default 0 so it doesn't run when we don't want it to
    double feederMotorPower = SmartDashboard.getNumber("feederMotorSpeed", 0f);
    targetFeederSpeed = feederMotorPower;
    // choose between PID control and just setting the power to the motor based on the constant
    // if(Constants.FeederConstants.usePID)
    //   setFeederVelocity(feederMotorPower);
    // else
    //   runFeeder(feederMotorPower);

    SmartDashboard.putNumber("feeder motor front actual RPM", feederMotorFront.getEncoder().getVelocity());
    SmartDashboard.putNumber("feeder motor back actual RPM", feederMotorBack.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
