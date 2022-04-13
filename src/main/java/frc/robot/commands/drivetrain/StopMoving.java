package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class StopMoving extends CommandBase {

    private SwerveDrivetrain drivetrain;

    public StopMoving (SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.stop();
    }
    
}
