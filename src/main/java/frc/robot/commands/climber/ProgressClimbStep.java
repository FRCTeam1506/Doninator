package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ProgressClimbStep extends CommandBase {

    private ClimberSubsystem climber;
    
    public ProgressClimbStep (ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(this.climber);
    }

    @Override
    public void initialize() {
        climber.progressClimb();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
