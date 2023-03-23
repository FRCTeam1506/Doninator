package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

public class Limelight3 extends SubsystemBase {
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tv;

    double target = 0;

    double ROTATION_K = 0.1;

    SwerveDrivetrain swerve = new SwerveDrivetrain();

    // Constants for control loop
    double kP = 0.1;  // Proportional constant
    double minSpeed = 0.1;  // Minimum speed for the robot to move

    public Limelight3() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("pipeline").setNumber(1);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
    }

    public void alignWithTarget() {
        double currentPipeline = table.getEntry("pipeline").getDouble(0.0);

        double x_offset = tx.getDouble(0.0);
        double y_offset = ty.getDouble(0.0);
        target = tv.getDouble(0.0);
        double area = ta.getDouble(0.0);


        System.out.println("align with target " + target);
        System.out.println("current pipeline value " + currentPipeline);
        System.out.println("xoffset: " + x_offset);
        table.getEntry("ledMode").setNumber(3);

        if (target == 1) {
            while(Math.abs(x_offset) > 0.5){
                
                boolean plusminus;
                if(x_offset > 0){
                    plusminus = true;
                }
                else{
                    plusminus = false;
                }

                x_offset = tx.getDouble(0.0);

                double divisor = 1/x_offset*200;

                // Adjust rotation to align the robot with the target
                double rotation = x_offset * ROTATION_K;
                rotation = 0.2 * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;

                Translation2d translation = new Translation2d(0, 0).times(Constants.SwerveDrivetrain.MAX_SPEED/-divisor);

                if(plusminus == true){
                    translation = new Translation2d(0, 0).times(Constants.SwerveDrivetrain.MAX_SPEED/divisor);
                }
                rotation = 0.2 * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
                
                
                
                if(plusminus == true){
                    swerve.drive(translation, -rotation, false, true);
                }
                else{
                    swerve.drive(translation, rotation, false, true); 
                }


                // Drive the robot with zero translation and the calculated rotation
                // swerve.drive(new Translation2d(0.1,0.1), 0.1, false, true);
                x_offset = tx.getDouble(0.0);



            }
        } 
        // else {
        //     // while (target == 0) {
        //     //     try {
        //     //         Thread.sleep(5);
        //     //     } catch (InterruptedException e) {
        //     //         e.printStackTrace();
        //     //     }
        //     }
        //     alignWithTarget();
        // }
    }

    public void alignWithTarget2() {
        Translation2d translation = new Translation2d(0, 0).times(Constants.SwerveDrivetrain.MAX_SPEED / -5);
        for (int i = 0; i < 500; i++) {
            swerve.drive(translation, 6, false, true);
            try {
                Thread.sleep(100); // pause for 10 milliseconds
            } catch (InterruptedException e) {
                // handle the exception by printing the stack trace
                e.printStackTrace();
            }
        }
    }
    

    @Override
    public void periodic() {

        // System.out.println(target);
        //read values periodically
        // System.out.println("Periodic");
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
