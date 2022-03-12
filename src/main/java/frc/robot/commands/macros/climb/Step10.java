package frc.robot.commands.macros.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class Step10 extends CommandBase {

    private ClimberSubsystem climber;

    public Step10 (ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(this.climber);
    }

    @Override
    public void execute() {
        climber.setMotorPosition(-1_000.0);
    }
    
}
