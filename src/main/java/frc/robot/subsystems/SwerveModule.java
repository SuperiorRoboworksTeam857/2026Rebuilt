package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
//import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkFlexUtil;
import frc.lib.util.CANSparkFlexUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private SparkFlex angleMotor;
  private SparkFlex driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController angleController;
  private final SparkFlexConfig driveConfig;
  private final SparkFlexConfig angleConfig;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new SparkFlex(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getClosedLoopController();
    angleConfig = new SparkFlexConfig();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new SparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();
    driveConfig = new SparkFlexConfig();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous
    // controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void resetToAbsolute() {
    double absolutePosition = getCanCoder().getRotations();// - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    //
    // angleEncoder.configFactoryDefault();
    // REV: Removes burnFlash() and restoreFactoryDefaults(). Use the ResetMode and
    // PersistMode options in SparkBase.configure() instead.

    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);

    CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();

    swerveCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    swerveCanCoderConfig.MagnetSensor.SensorDirection = 
      Constants.Swerve.canCoderInvert ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
    swerveCanCoderConfig.MagnetSensor.MagnetOffset = angleOffset.getRotations();

    angleEncoder.getConfigurator().apply(swerveCanCoderConfig);    
  }

  private void configAngleMotor() {
    // old
    // angleMotor.restoreFactoryDefaults();

    CANSparkFlexUtil.setCANSparkFlexBusUsage(angleMotor, Usage.kPositionOnly);
    Timer.delay(1);
    resetToAbsolute();

    // new
    angleConfig.inverted(Constants.Swerve.angleInvert);
    angleConfig.idleMode(Constants.Swerve.angleNeutralMode);
    angleConfig.encoder.positionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleConfig.smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    angleConfig.voltageCompensation(Constants.Swerve.voltageComp);
    angleConfig.closedLoop.p(Constants.Swerve.angleKP);
    angleConfig.closedLoop.i(Constants.Swerve.angleKI);
    angleConfig.closedLoop.d(Constants.Swerve.angleKD);
    angleConfig.closedLoop.velocityFF(Constants.Swerve.angleKFF);

    angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // angleMotor.configure(angleConfig,ResetMode.,PersistMode.); ADD RESETMODE AND
    // PERSIST MODE CONFIG
  }

  private void configDriveMotor() {
    // driveMotor.restoreFactoryDefaults();
    // old
    CANSparkFlexUtil.setCANSparkFlexBusUsage(driveMotor, Usage.kAll);
    driveConfig.smartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveConfig.inverted(Constants.Swerve.driveInvert);
    driveConfig.idleMode(Constants.Swerve.driveNeutralMode);
    driveConfig.encoder.velocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveConfig.encoder.positionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    // driveController.setP(Constants.Swerve.angleKP);
    // driveController.setI(Constants.Swerve.angleKI);
    // driveController.setD(Constants.Swerve.angleKD);
    // driveController.setFF(Constants.Swerve.angleKFF);
    driveConfig.voltageCompensation(Constants.Swerve.voltageComp);
    // driveMotor.burnFlash();

    // new
    driveConfig.closedLoop.p(Constants.Swerve.angleKP);
    driveConfig.closedLoop.i(Constants.Swerve.angleKI);
    driveConfig.closedLoop.d(Constants.Swerve.angleKD);
    driveConfig.closedLoop.velocityFF(Constants.Swerve.angleKFF);

    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    Timer.delay(1);
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          ClosedLoopSlot.kSlot0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    angleController.setSetpoint(angle.getRotations(), ControlType.kPosition);
    lastAngle = angle;
  }

  public void setAngleForX(double angle) {
    SwerveModuleState desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(angle));
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    driveMotor.set(desiredState.speedMetersPerSecond);
    angleController.setSetpoint(desiredState.angle.getRotations(), ControlType.kPosition);
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromRotations(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    SmartDashboard.putNumber("angleEncoder position " + moduleNumber, angleEncoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("angleOffset degrees " + moduleNumber, angleOffset.getDegrees());

    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromRotations(angleEncoder.getPosition().getValueAsDouble()));
  }
}
