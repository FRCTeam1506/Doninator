package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class BeginClimb extends CommandBase {

    private ClimberSubsystem climber;

    public BeginClimb (ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(this.climber);
    }

    /*  
        bring turret position to 0
        set turret hood to down
        set climb starting config: 
            - extendo: extended
            - leanboi: retracted
            - trigger: retracted
    */

}
