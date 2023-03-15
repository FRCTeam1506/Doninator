package frc.robot.commands.drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.lang.Math;

public class SwerveTeleop extends CommandBase {

    private static final double DEADBAND = 0.08;

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private SwerveDrivetrain s_Swerve;
    private PS4Controller controller;

    public SwerveTeleop (SwerveDrivetrain s_Swerve, PS4Controller controller, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        // double yAxis = -controller.getLeftY();
        // double xAxis = -controller.getLeftX();
        // double rAxis = -controller.getRightX();
        double yAxis = -controller.getRawAxis(1); //1.55
        double xAxis = -controller.getRawAxis(0); //1.49
        double rAxis = -controller.getRawAxis(2); //1.2
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < DEADBAND) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < DEADBAND) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < DEADBAND) ? 0 : rAxis;

        yAxis = Math.abs(yAxis)*yAxis;
        xAxis = Math.abs(xAxis)*xAxis;

        // yAxis = Math.pow(Math.abs(yAxis), 1.4);
        // xAxis = Math.pow(Math.abs(xAxis), 1.4);

        translation = new Translation2d(yAxis, xAxis).times(Constants.SwerveDrivetrain.MAX_SPEED);
        rotation = rAxis * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
