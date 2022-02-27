package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimberMatchSC extends CommandBase {
    
    private ClimberSubsystem climber;

    public SetClimberMatchSC (ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(this.climber);
    }

}
