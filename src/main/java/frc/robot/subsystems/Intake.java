// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private TalonSRX intakePowerMotor = new TalonSRX(Constants.IntakeConstants.intakePowerMotor);

  public Intake() {
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


  @Override
  public void periodic(){}
}
