package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RegressClimber extends CommandBase {

    private ClimberSubsystem climber;

    public RegressClimber (ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(this.climber);
    }

    @Override
    public void initialize() {
        climber.regressClimb();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
