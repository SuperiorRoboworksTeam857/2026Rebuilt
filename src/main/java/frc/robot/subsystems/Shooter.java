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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Shooter extends SubsystemBase {
  // 2.9.26
  // instead of using these direct motor controls, we also want to include a PID loop controller
  // we can setup a config and apply it to BOTH motors (as it *should* be the same)
  // then, this allows us to make a setpoint so that the motors can reach (and maintain the points themselves)
  private SparkFlex shooterMotor1 = new SparkFlex(Constants.ShooterConstants.shooterMotor1, MotorType.kBrushless);
  private SparkFlex shooterMotor2 = new SparkFlex(Constants.ShooterConstants.shooterMotor2, MotorType.kBrushless);
  private SparkFlex turretMotor = new SparkFlex(Constants.ShooterConstants.turretMotor, MotorType.kBrushless);

  // then we want to define our configuration
  // and have it setup to be the default
  // one for BOTH motors should be fine
  private SparkFlexConfig shooterMotorConfig1 = new SparkFlexConfig();
  private SparkFlexConfig shooterMotorConfig2 = new SparkFlexConfig();
  private double targetShooterSpeed = 0;
  private SparkFlexConfig turretMotorConfig = new SparkFlexConfig();
  // then we have the actual closed loop controllers
  // these ensure that we are running at the expected setpoint (velocity, position, etc.)
  private SparkClosedLoopController shooterMotor1Controller = shooterMotor1.getClosedLoopController();
  private SparkClosedLoopController shooterMotor2Controller = shooterMotor2.getClosedLoopController();
  private SparkClosedLoopController turretMotorController = turretMotor.getClosedLoopController();
  // these basically just do the same thing as .set()
  // but we are doing a .setSetpoint
  // - the controller takes in the current encoder value and the P I D constants
  // - the controller does some fancy math to determine what work needs to be done to get to our setpoint
  // - the controller instructs the motor to do so in a regular periodic

  Rotation2d shootRotationInRobotCoords;

  private final SwerveSubsystem s_swerve;

  // Example LUT (distance in meters, shooter RPM)
  private static final double minimumShootingDistance = 2.0;
  private static final double maximumDistanceForCafeteria = 4.0;
  private static final InterpolatingDoubleTreeMap SHOOTER_RPM_MAP = new InterpolatingDoubleTreeMap();
  static {
    SHOOTER_RPM_MAP.put(2.0, 2600.0);
    SHOOTER_RPM_MAP.put(2.5, 2800.0);
    SHOOTER_RPM_MAP.put(3.0, 3000.0);
    SHOOTER_RPM_MAP.put(3.5, 3200.0);
    SHOOTER_RPM_MAP.put(4.0, 3400.0);
    SHOOTER_RPM_MAP.put(4.5, 3600.0);
    SHOOTER_RPM_MAP.put(5.0, 3800.0);
    SHOOTER_RPM_MAP.put(5.5, 4000.0);
    SHOOTER_RPM_MAP.put(6.0, 4200.0);
    SHOOTER_RPM_MAP.put(6.5, 4400.0);
    SHOOTER_RPM_MAP.put(7.0, 4600.0);
    SHOOTER_RPM_MAP.put(7.5, 4800.0);
    SHOOTER_RPM_MAP.put(8.0, 5000.0);
    SHOOTER_RPM_MAP.put(8.5, 5200.0);
    SHOOTER_RPM_MAP.put(9.0, 5400.0);
    SHOOTER_RPM_MAP.put(9.5, 5600.0);
    SHOOTER_RPM_MAP.put(10.0, 5800.0);
    SHOOTER_RPM_MAP.put(10.5, 6000.0);
  }

  private static final InterpolatingDoubleTreeMap SHOOTER_TOF_MAP = new InterpolatingDoubleTreeMap();
  static {
    SHOOTER_TOF_MAP.put(2.0, 1.0);
    SHOOTER_TOF_MAP.put(2.5, 1.0);
    SHOOTER_TOF_MAP.put(3.0, 1.0);
    SHOOTER_TOF_MAP.put(3.5, 0.8);
    SHOOTER_TOF_MAP.put(4.0, 0.8);
    SHOOTER_TOF_MAP.put(10.5, 0.8); // who knows?
  }
  // m/s -> distance
  private static final InterpolatingDoubleTreeMap INVERSE_SHOOTER_SPEED_MAP = new InterpolatingDoubleTreeMap();
  static {
    INVERSE_SHOOTER_SPEED_MAP.put(2.0 / 1.0, 2.0);
    INVERSE_SHOOTER_SPEED_MAP.put(2.5 / 1.0, 2.5);
    INVERSE_SHOOTER_SPEED_MAP.put(3.0 / 1.0, 3.0);
    INVERSE_SHOOTER_SPEED_MAP.put(3.5 / 0.8, 3.5);
    INVERSE_SHOOTER_SPEED_MAP.put(4.0 / 0.8, 4.0);
    
    INVERSE_SHOOTER_SPEED_MAP.put(5.0 / 0.8, 5.0);
    INVERSE_SHOOTER_SPEED_MAP.put(6.0 / 0.8, 6.0);
    INVERSE_SHOOTER_SPEED_MAP.put(7.0 / 0.8, 7.0);
    INVERSE_SHOOTER_SPEED_MAP.put(8.0 / 0.8, 8.0);
    INVERSE_SHOOTER_SPEED_MAP.put(9.0 / 0.8, 9.0);
    INVERSE_SHOOTER_SPEED_MAP.put(10.5 / 0.8, 10.5);
    //INVERSE_SHOOTER_SPEED_MAP.put(4.0 / 1.25, 4.0);
  }

  public Shooter(SwerveSubsystem swerve) {
    this.s_swerve = swerve;
    
    //SmartDashboard.putNumber("target shooter speed", 2600.0);

    // setup PID parameters
    // this takes in the PID parameters from constants and applies it to the config
    // applying it to the configuration ALONE won't change the motor
    // we still have to connect the config object to the motor
    shooterMotorConfig1.closedLoop
      .p(Constants.ShooterConstants.shooterKP)
      .i(Constants.ShooterConstants.shooterKI)
      .d(Constants.ShooterConstants.shooterKD)
      .feedForward.kV(Constants.ShooterConstants.shooterKV);
    shooterMotorConfig2.apply(shooterMotorConfig1);
    // (hint: mess with these in the constants file)
    turretMotorConfig.closedLoop
      .p(Constants.ShooterConstants.turretKP)
      .i(Constants.ShooterConstants.turretKI)
      .d(Constants.ShooterConstants.turretKD)
      .feedForward.kV(Constants.ShooterConstants.turretKV);
    turretMotorConfig.encoder
      .positionConversionFactor(Constants.ShooterConstants.turretPositionFactor);
    turretMotorConfig.smartCurrentLimit(30);
    
    shooterMotorConfig2.inverted(true);

    shooterMotorConfig1.idleMode(IdleMode.kCoast);
    shooterMotorConfig2.idleMode(IdleMode.kCoast);

    // now APPLY the configuration to the shooter motors
    // do both
    shooterMotor1.configure(shooterMotorConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterMotor2.configure(shooterMotorConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turretMotor.configure(turretMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // these last two parameters basically say:
    // 1: reset the configuration safely (if it fails, then don't do something CRAZY)
    // 2: persist even if the robot power goes off (do we want these changes to be temporary)

    //turretMotor.getEncoder().setPosition(0);
  }
  public void setTurretPosition(double position) {
    position = clamp(position,
                     Constants.ShooterConstants.turretMinLimit,
                     Constants.ShooterConstants.turretMaxLimit);
    turretMotorController.setSetpoint(position, ControlType.kPosition);
    
  }

  public void setShooterVelocity(double velocity){
    // similar to how we have to set two motor speeds in the runShooter() function
    // we need to set both of the controllers to the same setpoint
    // (it takes TWO motors to drive)

    // act on the shooterMotor1Controller and 2... the controller CONTROLS the motor
    shooterMotor1Controller.setSetpoint(velocity, ControlType.kVelocity);
    shooterMotor2Controller.setSetpoint(velocity, ControlType.kVelocity); // make sure VELOCITY :), not POSITION
  }

  public void runShooter(double speed) {
    shooterMotor1.set(speed * Constants.ShooterConstants.shooterSpeedMultiplier);
    shooterMotor2.set(speed * Constants.ShooterConstants.shooterSpeedMultiplier);
  }
  public void powerShooter(double speed) {
    if(speed != 0) {
      setShooterVelocity(speed);
    } else {
      runShooter(speed); // Let it coast down to zero rather that velocity controller forcing it down
    }
  }
  public void startShooter() {
    powerShooter(targetShooterSpeed); //Change Later with something fancier
  }
  public void stopShooter() {
    powerShooter(0);
  }

  @Override
  public void periodic() {
    Pose2d robotPose = s_swerve.getPose();
    Pose2d shooterTargetPose = whereToShootAt(robotPose);
    // get the VECTOR from the robot to the target (translation2d)
    // this is to get the SOTF
    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly



    // THIS ALL ASSUMES ROBOT IS BASICALLY STATIONARY
    boolean USE_SHOOT_ON_FLY = true;
    if (!USE_SHOOT_ON_FLY) {
      Translation2d shootDirection = shooterTargetPose.getTranslation().minus(robotPose.getTranslation());
      Rotation2d shootRotationInFieldCoords = shootDirection.getAngle();
      Rotation2d robotRotationInFieldCoords = robotPose.getRotation();
      shootRotationInRobotCoords = shootRotationInFieldCoords
                                  .minus(robotRotationInFieldCoords)
                                  .unaryMinus().plus(Rotation2d.k180deg); // shooter is "backwards" on robot, and opposite rotation dir

      s_swerve.field.getObject("target").setPose(shooterTargetPose);

      // Only command the turret if it can reach the desired position
      if (shootRotationInRobotCoords.getRotations() > Constants.ShooterConstants.turretMinLimit &&
          shootRotationInRobotCoords.getRotations() < Constants.ShooterConstants.turretMaxLimit) {
        setTurretPosition(shootRotationInRobotCoords.getRotations());
      }

      // Calculate shooter speed from distance to target using lookup table
      double distanceToGoal = shootDirection.getNorm();
      if (distanceToGoal >= minimumShootingDistance && distanceToGoal <= maximumDistanceForCafeteria) {
        targetShooterSpeed = SHOOTER_RPM_MAP.get(distanceToGoal);
      }
      else { // if too close, don't try to run shooter
        targetShooterSpeed = 0;
      }

      SmartDashboard.putBoolean("Too Close", distanceToGoal < minimumShootingDistance);
      SmartDashboard.putBoolean("Too Far", distanceToGoal > maximumDistanceForCafeteria);
    } else {
      ChassisSpeeds robotSpeed = s_swerve.getFieldVelocity();
      Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);

      // 1. LATENCY COMP
      double latency = 0.15; // s, Tuned constant
      Translation2d futurePos = robotPose.getTranslation().plus( robotVelVec.times(latency) );

      // 2. GET TARGET VECTOR
      Translation2d goalLocation = shooterTargetPose.getTranslation();
      Translation2d targetVec = goalLocation.minus(futurePos);
      double distanceToGoal = targetVec.getNorm();

      // 3. CALCULATE IDEAL SHOT (Stationary)
      // Note: This returns HORIZONTAL velocity component
      double idealHorizontalSpeed = distanceToGoal / SHOOTER_TOF_MAP.get(distanceToGoal);

      // 4. VECTOR SUBTRACTION
      Translation2d shotVec = targetVec.div(distanceToGoal).times(idealHorizontalSpeed).minus(robotVelVec);

      // 5. CONVERT TO CONTROLS
      double newHorizontalSpeed_mps = shotVec.getNorm();

      double effectiveDistanceToGoal = INVERSE_SHOOTER_SPEED_MAP.get(newHorizontalSpeed_mps);
      
      // Calculate shooter speed from distance to target using lookup table
      if (effectiveDistanceToGoal >= minimumShootingDistance && effectiveDistanceToGoal <= maximumDistanceForCafeteria) {
        targetShooterSpeed = SHOOTER_RPM_MAP.get(effectiveDistanceToGoal);
      }
      else { // if too close, don't try to run shooter
        targetShooterSpeed = 0;
      }

      // Set turret rotation
      shootRotationInRobotCoords = shotVec.getAngle()
                                  .minus(robotPose.getRotation())
                                  .unaryMinus().plus(Rotation2d.k180deg); // shooter is "backwards" on robot, and opposite rotation dir

      s_swerve.field.getObject("target").setPose(shooterTargetPose);

      // Only command the turret if it can reach the desired position
      if (shootRotationInRobotCoords.getRotations() > Constants.ShooterConstants.turretMinLimit &&
          shootRotationInRobotCoords.getRotations() < Constants.ShooterConstants.turretMaxLimit) {
        setTurretPosition(shootRotationInRobotCoords.getRotations());
      }

      SmartDashboard.putBoolean("Too Close", distanceToGoal < minimumShootingDistance);
      SmartDashboard.putBoolean("Too Far", distanceToGoal > maximumDistanceForCafeteria);
    }



    SmartDashboard.putNumber("turret angle (rotations)", turretMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("turret goal", shootRotationInRobotCoords.getRotations());


    // put both of these numbers on the smartdashboard
    SmartDashboard.putNumber("shooter 1 actual RPM", shooterMotor1.getEncoder().getVelocity());
    SmartDashboard.putNumber("shooter 2 actual RPM", shooterMotor2.getEncoder().getVelocity());


    SmartDashboard.putBoolean("On Target", isShooterOnTarget());
    SmartDashboard.putBoolean("At Speed", isShooterAtSpeed());


    // SmartDashboard.putBoolean("Too Close", distanceToGoal < minimumShootingDistance);
    // SmartDashboard.putBoolean("Too Far", distanceToGoal > maximumDistanceForCafeteria);

    
  }
  public boolean isShooterAtSpeed() {
    return Math.abs(shooterMotor1.getEncoder().getVelocity() - targetShooterSpeed) < 500
           && targetShooterSpeed > 500;
  }
  public boolean isShooterOnTarget() {
    double turretError = turretMotor.getEncoder().getPosition() - shootRotationInRobotCoords.getRotations();
    SmartDashboard.putNumber("turret error", turretError);

    return Math.abs(turretError) < 0.02;

    // typical error seems to be 0.004 or 0.007 rotations
  }
  public void runShooterThenRest(Feeder feeder, Spindexer spindexer){
    powerShooter(targetShooterSpeed);
    if (isShooterAtSpeed() && isShooterOnTarget()){
      feeder.startFeeder();
      spindexer.startSpindexer();
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }


  public Pose2d whereToShootAt(Pose2d robotPose){
    Pose2d pose = Constants.ShooterConstants.blueDownCorner;

    if (robotIsInAllianceZone(robotPose)) {
      if (isBlueAlliance()) {
        pose = Constants.ShooterConstants.blueHubLocation;
      } else { // red alliance
        pose = Constants.ShooterConstants.redHubLocation;
      }
    } else { // not in alliance zone
      double y_meters = robotPose.getY();
      double y_inches = Units.metersToInches(y_meters);

      if (isBlueAlliance()) {
        if (y_inches < 158) {
          pose = Constants.ShooterConstants.blueDownCorner;
        } else {
          pose = Constants.ShooterConstants.blueUpCorner;
        }
      } else { // red alliance
        if (y_inches < 158) {
          pose = Constants.ShooterConstants.redDownCorner;
        } else {
          pose = Constants.ShooterConstants.redUpCorner;
        }
      }
    }

    return pose;
  }

  public boolean robotIsInAllianceZone(Pose2d robotPose){
    double x_meters = robotPose.getX();
    double x_inches = Units.metersToInches(x_meters);

    boolean inAllianceZone = false;
    if (isBlueAlliance()) {
      if (x_inches < 160) {
        inAllianceZone = true;
      }
    } else {
      if (x_inches > 490) {
        inAllianceZone = true;
      }
    }

    return inAllianceZone;
  }

  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue;
  }
}
