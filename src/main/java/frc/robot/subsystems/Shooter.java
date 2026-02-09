// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.IntakeConstants;
import frc.robot.Constants.Swerve.ShooterConstants;

public class Shooter extends SubsystemBase {

  private SparkFlex shooterMotor1 = new SparkFlex(Constants.ShooterConstants.shooterMotor1, MotorType.kBrushless);
  private SparkFlex shooterMotor2 = new SparkFlex(Constants.ShooterConstants.shooterMotor2, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public Shooter() {

    SmartDashboard.putNumber("shooterMotorSpeed", 0);

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void runShooter(double speed) {
    shooterMotor1.set(speed * ShooterConstants.shooterSpeedMultiplier);
    shooterMotor2.set(speed * ShooterConstants.shooterSpeedMultiplier);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double shooterMotorPower = SmartDashboard.getNumber("shooterMotorSpeed", 0f);
    shooterMotor1.set(shooterMotorPower);
    shooterMotor2.set(shooterMotorPower);

    // shooterMotor1.setVoltage(shooterMotorPower);
    // shooterMotor2.setVoltage(shooterMotorPower);

    SmartDashboard.putNumber("shooter 1 actual RPM", shooterMotor1.getEncoder().getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
