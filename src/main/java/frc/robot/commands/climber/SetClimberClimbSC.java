package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class SetClimberClimbSC extends CommandBase {

    private ClimberSubsystem climber;

    public SetClimberClimbSC (ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(this.climber);
    }
    
}
