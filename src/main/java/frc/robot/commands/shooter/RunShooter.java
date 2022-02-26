
package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooter extends CommandBase {

    private ShooterSubsystem shooter;

    private double velocity_rpm;
    
    public RunShooter (ShooterSubsystem shooter, double velocity_rpm) {
        this.shooter = shooter;
        addRequirements(shooter);

        this.velocity_rpm = velocity_rpm;
    }

    @Override
    public void execute() {
        shooter.shoot(velocity_rpm);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
