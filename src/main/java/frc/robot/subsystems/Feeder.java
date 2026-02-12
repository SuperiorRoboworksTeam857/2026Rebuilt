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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

  // one feederMotor in the middle that spins to bring the fuel cells around
  private SparkFlex feederMotor = new SparkFlex(Constants.FeederConstants.feederWheel, MotorType.kBrushless);
  private SparkFlexConfig feederConfig = new SparkFlexConfig(); // to handle the PID loop of the middle loop
  private SparkClosedLoopController feederController = feederMotor.getClosedLoopController();

  /** Creates a new ExampleSubsystem. */
  public Feeder() {
    SmartDashboard.putNumber("feederMotorSpeed", 0);

    // setup PID parameters
    feederConfig.closedLoop
      .p(Constants.FeederConstants.feederKP)
      .i(Constants.FeederConstants.feederKI)
      .d(Constants.FeederConstants.feederKD);

    feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setFeederVelocity(double velocity){
    feederController.setSetpoint(velocity, ControlType.kVelocity);
  }
  
  public void runFeeder(double speed) {
    feederMotor.set(speed * Constants.FeederConstants.feederSpeedMultiplier);
  }

  @Override
  public void periodic() {
    // grab from the dashboard the speed for the feeder and set it
    // default 0 so it doesn't run when we don't want it to
    double feederMotorPower = SmartDashboard.getNumber("feederMotorSpeed", 0f);
    // choose between PID control and just setting the power to the motor based on the constant
    if(Constants.FeederConstants.usePID)
      setFeederVelocity(feederMotorPower);
    else
      runFeeder(feederMotorPower);

    SmartDashboard.putNumber("feeder motor actual RPM", feederMotor.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
