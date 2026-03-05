// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public Limelight() {}

  private boolean isReefAprilTagValid = false;

  public boolean getIsReefAprilTagValid(){
    return isReefAprilTagValid;
  }

  public void setIsReefAprilTagValid(boolean isValid){
    isReefAprilTagValid = isValid;
  }

  @Override
  public void periodic() {
    int tag = aprilTagID();
    boolean isValid = aprilTagValid();

    SmartDashboard.putBoolean("AprilTag - isValid", isValid);
    SmartDashboard.putNumber("AprilTag - tag ID", tag);
    

    if(isValid){
        double distance = distanceToAprilTagMeters();
        SmartDashboard.putNumber("AprilTag - distance", distance);
    }else{
        SmartDashboard.putNumber("AprilTag - distance", -1);
    }
  }

  public int aprilTagID(){
    var limeLight = getLimelightValue("tid");
    return (int)limeLight;
  }
  public boolean aprilTagValid(){
    var tagValid = getLimelightValue("tv");
    if(tagValid == 1){
      return true;
    }
    else{
      return false;
    }
  }

  public double distanceToAprilTagMeters() {
    var pose = getLimelightArray("camerapose_targetspace");
    double tx = pose[0];
    double tz = pose[2];

    return Math.sqrt(tx*tx + tz*tz);
  }

  public boolean isTurnedToTarget() {
    boolean ans = false;

    if (Math.abs(getLimelightValue("tx")) < 4.0) {
      ans = true;
    }

    return ans;
  }

  public void turnOnDriverCam() {
    setLimelightValue("camMode", 1);
  }

  public void turnOffDriverCam() {
    setLimelightValue("camMode", 0);
  }

  public void toggleDriverCam() {
    double currentMode = getLimelightValue("camMode");

    if (currentMode == 0) {
      setLimelightValue("camMode", 1);
    } else if (currentMode == 1) {
      setLimelightValue("camMode", 0);
    }
  }

  public void enableLimelight(boolean enabled) {
    if (enabled) {
      setLimelightValue("ledMode", 0);
    } else {
      setLimelightValue("ledMode", 1);
    }
  }

  public double getLimelightValue(String entry) {

    double value = 0.0;

    if (entry.equals("tDistance")) {
      // value = (LimelightConstants.kTargetHeight - LimelightConstants.kLimelightHeight) /
      // Math.tan(Math.toRadians(LimelightConstants.kLimelightAngle + getLimelightValue("ty")));
    } else {
      value =
          NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).getDouble(0.0);
    }

    return value;
  }

  public void setLimelightValue(String entry, double value) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).setDouble(value);
  }

  public double[] getLimelightArray(String entry) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).getDoubleArray(new double[6]);
  }

  public void setLimelightArray(String entry, double[] value) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry(entry).setDoubleArray(value);
  }

  public enum Pipeline {
    //RetroTape,
    AprilTags
  }

  public void setPipeline(Pipeline pipeline) {
    double value = 0.0;
    if (pipeline == Pipeline.AprilTags) {
      value = 0;
    }
    setLimelightValue("pipeline", value);
  }
}