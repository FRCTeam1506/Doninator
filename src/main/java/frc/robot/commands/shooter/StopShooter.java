package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooter extends CommandBase {
    
    private ShooterSubsystem shooter;

    public StopShooter (ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    @Override
    public void execute() {
        shooter.stop();
    }

}
