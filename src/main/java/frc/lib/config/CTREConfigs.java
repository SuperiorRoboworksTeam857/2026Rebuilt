package frc.lib.config;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public final class CTREConfigs {
  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new CANcoderConfiguration();

    swerveCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    swerveCanCoderConfig.MagnetSensor.SensorDirection = 
      Constants.Swerve.canCoderInvert ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

    //TODO
    /* Swerve CANCoder Configuration */
    // swerveCanCoderConfig.initializationStrategy =
    //     SensorInitializationStrategy.BootToAbsolutePosition;
    // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
  }
}