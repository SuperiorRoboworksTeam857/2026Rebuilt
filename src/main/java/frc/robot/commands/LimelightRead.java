package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Limelight.Pipeline;

public class LimelightRead extends Command {

    private final Limelight s_Limelight;
    public LimelightRead(Limelight limelight){
        this.s_Limelight = limelight;
        addRequirements(this.s_Limelight);
    }

    @Override
    public void initialize(){
        this.s_Limelight.setPipeline(Pipeline.AprilTags);
    }
    
    @Override
    public void execute(){
        int tag = s_Limelight.aprilTagID();
        boolean isValid = s_Limelight.aprilTagValid();

        SmartDashboard.putBoolean("AprilTag - isValid", isValid);
        SmartDashboard.putNumber("AprilTag - tag ID", tag);

        if(isValid){
            double distance = s_Limelight.distanceToAprilTagMeters();
            SmartDashboard.putNumber("AprilTag - distance", distance);
        }else{
            SmartDashboard.putNumber("AprilTag - distance", -1);
        }
    }
}
