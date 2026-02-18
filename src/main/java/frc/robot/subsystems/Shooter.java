// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  // 2.9.26
  // instead of using these direct motor controls, we also want to include a PID loop controller
  // we can setup a config and apply it to BOTH motors (as it *should* be the same)
  // then, this allows us to make a setpoint so that the motors can reach (and maintain the points themselves)
  private SparkFlex shooterMotor1 = new SparkFlex(Constants.ShooterConstants.shooterMotor1, MotorType.kBrushless);
  private SparkFlex shooterMotor2 = new SparkFlex(Constants.ShooterConstants.shooterMotor2, MotorType.kBrushless);

  // then we want to define our configuration
  // and have it setup to be the default
  // one for BOTH motors should be fine
  private SparkFlexConfig shooterMotorConfig1 = new SparkFlexConfig();
  private SparkFlexConfig shooterMotorConfig2 = new SparkFlexConfig();
  private double targetShooterSpeed = 0.5;
  // then we have the actual closed loop controllers
  // these ensure that we are running at the expected setpoint (velocity, position, etc.)
  private SparkClosedLoopController shooterMotor1Controller = shooterMotor1.getClosedLoopController();
  private SparkClosedLoopController shooterMotor2Controller = shooterMotor2.getClosedLoopController();
  // these basically just do the same thing as .set()
  // but we are doing a .setSetpoint
  // - the controller takes in the current encoder value and the P I D constants
  // - the controller does some fancy math to determine what work needs to be done to get to our setpoint
  // - the controller instructs the motor to do so in a regular periodic

  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    SmartDashboard.putNumber("shooterMotorSpeed", 0);

    // setup PID parameters
    // this takes in the PID parameters from constants and applies it to the config
    // applying it to the configuration ALONE won't change the motor
    // we still have to connect the config object to the motor
    shooterMotorConfig1.closedLoop
      .p(Constants.ShooterConstants.shooterKP)
      .i(Constants.ShooterConstants.shooterKI)
      .d(Constants.ShooterConstants.shooterKD);
    shooterMotorConfig2.apply(shooterMotorConfig1);
    // (hint: mess with these in the constants file)

    shooterMotorConfig2.inverted(true);

    shooterMotorConfig1.idleMode(IdleMode.kCoast);
    shooterMotorConfig2.idleMode(IdleMode.kCoast);

    // now APPLY the configuration to the shooter motors
    // do both
    shooterMotor1.configure(shooterMotorConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterMotor2.configure(shooterMotorConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // these last two parameters basically say:
    // 1: reset the configuration safely (if it fails, then don't do something CRAZY)
    // 2: persist even if the robot power goes off (do we want these changes to be temporary)
  }

  public void setShooterVelocity(double velocity){
    // similar to how we have to set two motor speeds in the runShooter() function
    // we need to set both of the controllers to the same setpoint
    // (it takes TWO motors to drive)

    // act on the shooterMotor1Controller and 2... the controller CONTROLS the motor
    shooterMotor1Controller.setSetpoint(velocity, ControlType.kVelocity);
    shooterMotor2Controller.setSetpoint(velocity, ControlType.kVelocity); // make sure VELOCITY :), not POSITION
    // position will be useful elsewhere
  }

  public void runShooter(double speed) {
    shooterMotor1.set(speed * Constants.ShooterConstants.shooterSpeedMultiplier);
    shooterMotor2.set(speed * Constants.ShooterConstants.shooterSpeedMultiplier);
  }
  public void powerShooter(double speed) {
    if(Constants.ShooterConstants.usePID)
      setShooterVelocity(speed);
    else
      runShooter(speed);
  }
  public void startShooter() {
    powerShooter(targetShooterSpeed); //Change Later with something fancier
  }
  public void stopShooter() {
    powerShooter(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // default system
    double shooterMotorPower = SmartDashboard.getNumber("shooterMotorSpeed", 0f);
    targetShooterSpeed = shooterMotorPower;
    // now apply this to either the controller or basic speed
    if(Constants.ShooterConstants.usePID) {
      setShooterVelocity(shooterMotorPower);
    }
    else {
      runShooter(shooterMotorPower);
    }

    // put both of these numbers on the smartdashboard
    SmartDashboard.putNumber("shooter 1 actual RPM", shooterMotor1.getEncoder().getVelocity());
    SmartDashboard.putNumber("shooter 2 actual RPM", shooterMotor2.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
