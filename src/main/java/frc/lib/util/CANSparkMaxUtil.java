package frc.lib.util;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel;

/** Sets motor usage for a Spark Max motor controller */
public class CANSparkMaxUtil {
  public enum Usage {
    kAll,
    kPositionOnly,
    kVelocityOnly,
    kMinimal
  };

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   * @param enableFollowing Whether to enable motor following.
   */
  public static void setCANSparkMaxBusUsage(
      SparkMax motor, Usage usage, boolean enableFollowing) {
      SparkMaxConfig config = new SparkMaxConfig();
    if (enableFollowing) {
      config.signals.faultsPeriodMs(10); // Status 0
    } else {
      config.signals.faultsPeriodMs(500); // Status 0
    }

    if (usage == Usage.kAll) {
      config.signals.primaryEncoderVelocityPeriodMs(20) // Status 1
                    .primaryEncoderPositionPeriodMs(20) // Status 2
                    .analogVoltagePeriodMs(50); // Status 3
    } else if (usage == Usage.kPositionOnly) {
      config.signals.primaryEncoderVelocityPeriodMs(500) // Status 1
                    .primaryEncoderPositionPeriodMs(20) // Status 2
                    .analogVoltagePeriodMs(500); // Status 3
    } else if (usage == Usage.kVelocityOnly) {
      config.signals.primaryEncoderVelocityPeriodMs(20) // Status 1
                    .primaryEncoderPositionPeriodMs(500) // Status 2
                    .analogVoltagePeriodMs(500); // Status 3
    } else if (usage == Usage.kMinimal) {
      config.signals.primaryEncoderVelocityPeriodMs(500) // Status 1
                    .primaryEncoderPositionPeriodMs(500) // Status 2
                    .analogVoltagePeriodMs(500); // Status 3
    }
  }

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a CANSparkMax is
   *     constructed.
   */
  public static void setCANSparkMaxBusUsage(SparkMax motor, Usage usage) {
    setCANSparkMaxBusUsage(motor, usage, false);
  }
}