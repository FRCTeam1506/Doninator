package frc.robot.commands.drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveTeleop extends CommandBase {

    private static final double DEADBAND = 0.1;

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

        double xAxis, yAxis, rAxis;
        if (controller.getL1Button()) {
            xAxis =  controller.getRawAxis(1) * 0.61; // 0.55 0.48
            yAxis =  controller.getRawAxis(0) * 0.71; // 0.55 0.48
            rAxis = -controller.getRawAxis(2) * 0.85;
        } else {
            xAxis =  controller.getRawAxis(1) * 0.49;
            yAxis =  controller.getRawAxis(0) * 0.55;
            rAxis = -controller.getRawAxis(2) * 0.77;
        }

        /* Deadbands */
        xAxis = (Math.abs(xAxis) < DEADBAND) ? 0 : xAxis;
        yAxis = (Math.abs(yAxis) < DEADBAND) ? 0 : yAxis;
        rAxis = (Math.abs(rAxis) < DEADBAND) ? 0 : rAxis;

        translation = new Translation2d(xAxis, yAxis).times(Constants.SwerveDrivetrain.MAX_SPEED);
        rotation = rAxis * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
