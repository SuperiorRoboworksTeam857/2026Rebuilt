package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Swerve extends SubsystemBase {
  private final AHRS gyro;

  private SwerveDrivePoseEstimator swervePoseEstimator;
  private SwerveModule[] mSwerveMods;

  private RobotConfig robotConfig;

  

  

  public Field2d field;

  public Swerve() {
    gyro = new AHRS(NavXComType.kMXP_SPI);
    zeroGyro();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    swervePoseEstimator =
        new SwerveDrivePoseEstimator(
          Constants.Swerve.swerveKinematics,
          getYaw(),
          getPositions(),
          new Pose2d(),
          VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
    LimelightHelpers.setCameraPose_RobotSpace("limelight", Units.inchesToMeters(2-27*0.5), Units.inchesToMeters(0.25), 0.384, 0.0, 0.0, 180.0);

    // Set AprilTag offset tracking point (meters)
    LimelightHelpers.setFiducial3DOffset("limelight", 0.0, 0.0, 0.384);

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    // initialize from GUI Settings of Pathplanner
    try{
      robotConfig = RobotConfig.fromGUISettings();
    }catch(Exception e){
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    // using new docs for PathPlanner changes: https://pathplanner.dev/pplib-getting-started.html#install-pathplannerlib
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(4.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(1.5, 0.0, 0.0) // Rotation PID constants
            ),
            robotConfig,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  public void setValidAprilTags() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red)) {
      int[] validIDs = {6,7,8,9,10,11};
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
    } else {
      int[] validIDs = {17,18,19,20,21,22};
      LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
    }
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

    if (swervePoseEstimator == null) {
      return;
    }

    double rotationDegrees = 0;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red)) {
      rotationDegrees = 180;
    }

    Rotation2d robotOrientation = swervePoseEstimator.getEstimatedPosition().getRotation().plus(Rotation2d.fromDegrees(rotationDegrees));

    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, robotOrientation)
                : new ChassisSpeeds(-translation.getX(), -translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    var states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(robotRelativeSpeeds);

    //SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);

    setModuleStates(states);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(mSwerveMods[0].getState(),
                                                             mSwerveMods[1].getState(),
                                                             mSwerveMods[2].getState(),
                                                             mSwerveMods[3].getState());
  }
  

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], true); // false
    }
  }

  public Pose2d getPose() {
    if (swervePoseEstimator == null) {
      return new Pose2d(0,0, new Rotation2d(0));
    }
    SmartDashboard.putNumber("pose X", swervePoseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("pose Y", swervePoseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("gyro angle", gyro.getAngle());
    
    // SmartDashboard.putNumber("gyro filtered X", gyro.getXFilteredAccelAngle()); // loops between
    // about 14...0...360...346
    // SmartDashboard.putNumber("gyro filtered Y", gyro.getYFilteredAccelAngle()); // forward and
    // back leveling
    // 0-14, drive forward, 346-360 drive backward

    SmartDashboard.putNumber("gyro pitch", gyro.getPitch());
    SmartDashboard.putNumber("gyro roll", gyro.getRoll());
    SmartDashboard.putNumber("pitch rate", getPitchRate());

    return swervePoseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    if (swervePoseEstimator == null) {
      return;
    }
    swervePoseEstimator.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "position: module " + mod.moduleNumber, mod.getPosition().distanceMeters);
      SmartDashboard.putNumber(
          "angle: module " + mod.moduleNumber, mod.getPosition().angle.getDegrees());
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.reset();
    double rotationDegrees = 0;
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red)) {
      rotationDegrees = 180;
    }

    Pose2d oldPose = getPose();
    Pose2d pose = new Pose2d(oldPose.getX(), oldPose.getY(), Rotation2d.fromDegrees(rotationDegrees));
    resetPose(pose);
    
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getAngle())
        : Rotation2d.fromDegrees(gyro.getAngle());
  }

  public double getYawRate() {
    return gyro.getRawGyroZ();
  }

  // public double getXFilteredAccelAngle() {
  //   return gyro.getXFilteredAccelAngle();
  // }

  // public double getYFilteredAccelAngle() {
  //   return gyro.getYFilteredAccelAngle();
  // }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getPitchRate() {
    return gyro.getRawGyroY();
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  // SmartDashboard.putNumber("gyro filtered X", gyro.getXFilteredAccelAngle()); // loops between
  // about 14...0...360...346
  // SmartDashboard.putNumber("gyro filtered Y", gyro.getYFilteredAccelAngle()); // forward and back
  // leveling
  // 0-14, drive forward, 346-360 drive backward

  public void setX() {
    mSwerveMods[0].setAngleForX(45);
    mSwerveMods[1].setAngleForX(-45);
    mSwerveMods[2].setAngleForX(-45);
    mSwerveMods[3].setAngleForX(45);
  }

  @Override
  public void periodic() {
    if (swervePoseEstimator == null) {
      return;
    }

    LimelightHelpers.SetRobotOrientation("limelight", swervePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
     
    // if our angular velocity is greater than 360 degrees per second, ignore vision updates
    boolean doRejectUpdate = false;
    if (Math.abs(gyro.getRate()) > 360) {
      doRejectUpdate = true;
    }
    if (mt2 == null) {
      doRejectUpdate = true;
    } else if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      swervePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      swervePoseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }

    swervePoseEstimator.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    SmartDashboard.putData("Field", field);

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
