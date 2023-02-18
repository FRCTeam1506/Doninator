package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class Limelight2 extends SubsystemBase {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    double target;


    double ROTATION_K = 0.01;

    SwerveDrivetrain swerve = new SwerveDrivetrain();

    // Constants for control loop
    double kP = 0.1;  // Proportional constant
    double minSpeed = 0.1;  // Minimum speed for the robot to move
    
    public void alignWithTarget() throws InterruptedException {

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //turn on LED's

        if(target!= 0){
        double x_offset = tx.getDouble(0.0);
    
        // Adjust rotation to align the robot with the target
        double rotation = x_offset * ROTATION_K;
    
        // Drive the robot with zero translation and the calculated rotation
        swerve.drive(new Translation2d(), rotation, false, true);
        }
        // else{
        //     while(target==0){
        //         wait(5, 0);
        //     }
        //     alignWithTarget();
        // }
    }
    

    @Override
    public void periodic() {
        //read values periodically
        double x_offset = tx.getDouble(0.0);
        double y_offset = ty.getDouble(0.0);
        target = tv.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x_offset);
        SmartDashboard.putNumber("LimelightY", y_offset);
        SmartDashboard.putNumber("Target Yes/No", target);
        SmartDashboard.putNumber("LimelightArea", area);
    }
}