package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

//! UNTESTED
public class RotateToAngle extends CommandBase {

    private SwerveDrivetrain drivetrain;
    private PIDController thetaController;
    private double degrees;

    public RotateToAngle (SwerveDrivetrain drivetrain, double thetaDegrees) {
        this.drivetrain = drivetrain;
        this.degrees = thetaDegrees;
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        thetaController = new PIDController(0.1, 0.0, 0.5);
        thetaController.enableContinuousInput(0, 360);
        thetaController.setTolerance(5.0);
    }

    @Override
    public void execute() {
        double setpoint = drivetrain.getPose().getRotation().getDegrees() - degrees;
        if (setpoint < 0) {
            setpoint = 360 - setpoint;
        }
        thetaController.setSetpoint(setpoint);
        double omega = -thetaController.calculate(drivetrain.getPose().getRotation().getDegrees());
        drivetrain.openLoopDrive(ChassisSpeeds.fromFieldRelativeSpeeds(
            0.0, 0.0, omega, drivetrain.getYaw()
        ));
        System.out.println(omega);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

}
