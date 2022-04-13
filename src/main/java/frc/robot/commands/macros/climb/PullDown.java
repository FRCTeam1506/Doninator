package frc.robot.commands.macros.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class PullDown extends CommandBase {

    private ClimberSubsystem climber;

    public PullDown (ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(this.climber);
    }

    @Override
    public void execute() {
        climber.setMotorPosition(-1_500.0, false);
    }
    
}
