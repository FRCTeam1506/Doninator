package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {

    public boolean aligned, isRefreshed;
    private Double x, y, area, targetDistance, previousX, previousY, previousDist;
    private boolean targetFound;
    private Integer pipeline;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    SwerveDrivetrain swerve = new SwerveDrivetrain();

    // Constants for control loop
    double kP = 0.1;  // Proportional constant
    double minSpeed = 0.1;  // Minimum speed for the robot to move
    
    public void turnToTarget() {
        double x = table.getEntry("tx").getDouble(0);  // Horizontal offset of the target
        double error = -x;  // Error is negative because we want to move in the opposite direction of the offset

        // Calculate the turn rate based on the error
        double turnRate = kP * error;

        // If the turn rate is too small, set it to the minimum speed in the correct direction
        if (Math.abs(turnRate) < minSpeed) {
            turnRate = Math.signum(turnRate) * minSpeed;
        }

        // Set the robot's speed and turn rate
        // swerve.driveCartesian(0, 0, turnRate);
    }


    // public enum Piplelines {
    //     NearTargeting,
    //     FarTargeting,
    //     Red,
    //     Green,
    //     Blue,
    //     Yellow
    // }

    public Limelight() {
        this.targetDistance = 1234.0;
        this.x = 0.0;
        this.previousDist = 5000.0;
    }

    // public void setPipeline(int index) {
    //     this.table.getEntry("pipeline").setNumber(index);
    // }

    // public void setPipeline(Piplelines pipe) {
    //     switch (pipe) {
    //         case NearTargeting:
    //             this.setPipeline(5);
    //             break;

    //         case FarTargeting:
    //             this.setPipeline(6);
    //             break;

    //         case Red:
    //             this.setPipeline(0);
    //             break;

    //         case Green:
    //             this.setPipeline(1);
    //             break;

    //         case Blue:
    //             this.setPipeline(2);
    //             break;

    //         case Yellow:
    //             this.setPipeline(3);
    //             break;
        
    //         default:
    //             this.setPipeline(5);
    //             break;
    //     }
    // }

    public void setTargetDistance(Double distance) {
        this.targetDistance = distance;
    }

    public Double getX() {
        return this.x;
    }

    public Double getY() {
        return this.y;
    }

    public Double getArea() {
        return this.area;
    }

    public int getPipeline() {
        return this.pipeline;
    }

    public boolean isRefreshed() {
        return this.isRefreshed;
    }

    public Double getDistance() {
        double val;
        // inches
        Double h2 = 96.0;
        Double h1 = 30.0;
        Double a1 = 1.0; // 0.258
        Double a2 = this.y;
        // return (h2 - h1) / Math.tan(a1 + a2);
        double dist = (h2 - h1) / (Math.tan((a1 + a2) * (Math.PI / 180)));
        if(dist > 0) {
            val = (double) Math.ceil((dist / 1000.0)) * 1000.0;
            if(Math.abs(val) < 1) this.previousDist = val;
        } else {
            val = this.previousDist;
        }
        return val;
    }

    public Double getTargetDistance() {
        return this.targetDistance;
    }

    public boolean isTargetFound() {
        return this.targetFound;
    }

    public boolean isAligned() {
        return this.aligned;
    }

    @Override
    public void periodic() {
        this.x = table.getEntry("tx").getDouble(0.0);
        this.y = table.getEntry("ty").getDouble(0.0);
        this.area = table.getEntry("ta").getDouble(0.0);
        this.targetFound = table.getEntry("tv").getNumber(0).intValue() == 1 ? true : false;
        // this.pipeline = table.getEntry("pipeline").getNumber(0).intValue();
        this.aligned = Math.abs(x) < Constants.Limelight.THRESHOLD ? true : false;
        this.isRefreshed = this.x != previousX || this.y != previousY ? true : false;
        this.previousX = this.x;
        this.previousY = this.y;
        // if(Math.abs(x) < Constants.Limelight.THRESHOLD) this.aligned = true;
        // else this.aligned = false;
        SmartDashboard.putNumber("[Limelight]-Target-Distance", getTargetDistance());
        SmartDashboard.putNumber("[Limelight]-Distance", getDistance());
    }
}