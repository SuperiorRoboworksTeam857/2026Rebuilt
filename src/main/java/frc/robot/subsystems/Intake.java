// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

public class Intake extends SubsystemBase {

  // one feederMotor in the middle that spins to bring the fuel cells around
  // private SparkFlex intakeMotorLeft = new
  // SparkFlex(Constants.IntakeConstants.intakeMotorLeft, MotorType.kBrushless);
  // private SparkFlex intakeMotorRight = new
  // SparkFlex(Constants.IntakeConstants.intakeMotorRight, MotorType.kBrushless);
  private TalonSRX intakeMotor = new TalonSRX(Constants.IntakeConstants.intakeMotorLeft);
  // private SparkFlexConfig intakeConfigLeft = new SparkFlexConfig(); // to
  // handle the PID loop of the middle loop
  // private SparkFlexConfig intakeConfigRight = new SparkFlexConfig(); // to
  // handle the PID loop of the middle loop
  // private SparkClosedLoopController intakeControllerLeft =
  // intakeMotorLeft.getClosedLoopController();
  // private SparkClosedLoopController intakeControllerRight =
  // intakeMotorRight.getClosedLoopController();
  private double targetIntakeSpeed = 0;

  /** Creates a new ExampleSubsystem. */
  public Intake() {
    SmartDashboard.putNumber("intakeMotorSpeed", 0);

    // setup PID parameters
    // intakeConfigLeft.closedLoop
    // .p(Constants.IntakeConstants.intakeKP)
    // .i(Constants.IntakeConstants.intakeKI)
    // .d(Constants.IntakeConstants.intakeKD);

    // intakeConfigRight.closedLoop
    // .p(Constants.IntakeConstants.intakeKP)
    // .i(Constants.IntakeConstants.intakeKI)
    // .d(Constants.IntakeConstants.intakeKD);

    // // feederConfigBack.inverted(true); --might change in future

    // intakeConfigLeft.idleMode(IdleMode.kCoast);
    // intakeConfigRight.idleMode(IdleMode.kCoast);

    // intakeMotorLeft.configure(intakeConfigLeft, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
    // intakeMotorRight.configure(intakeConfigRight, ResetMode.kResetSafeParameters,
    // PersistMode.kPersistParameters);
  }

  // public void setIntakeVelocity(double velocity){
  // intakeControllerLeft.setSetpoint(velocity, ControlType.kVelocity);
  // intakeControllerRight.setSetpoint(velocity, ControlType.kVelocity);
  // }

  public void runIntake(double speed) {
    // intakeMotorLeft.set(speed * Constants.IntakeConstants.intakeSpeedMultiplier);
    // intakeMotorRight.set(speed *
    // Constants.IntakeConstants.intakeSpeedMultiplier);
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void powerIntake(double speed) {
    runIntake(speed);

  }

  public void startIntake() {
    powerIntake(targetIntakeSpeed);
  }

  public void stopIntake() {
    powerIntake(0);
  }

  public void reverseIntake() {
    powerIntake(-targetIntakeSpeed);
  }

  @Override
  public void periodic() {
    // grab from the dashboard the speed for the feeder and set it
    // default 0 so it doesn't run when we don't want it to
    double intakeMotorPower = SmartDashboard.getNumber("intakeMotorSpeed", 0f);
    targetIntakeSpeed = intakeMotorPower;
    // choose between PID control and just setting the power to the motor based on
    // the constant
    // if(Constants.FeederConstants.usePID)
    // setIntakeVelocity(intakeMotorPower);
    // else
    // runIntake(intakeMotorPower);

    // SmartDashboard.putNumber("intake motor front actual RPM",
    // intakeMotorLeft.getEncoder().getVelocity());
    // SmartDashboard.putNumber("intake motor back actual RPM",
    // intakeMotorRight.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
