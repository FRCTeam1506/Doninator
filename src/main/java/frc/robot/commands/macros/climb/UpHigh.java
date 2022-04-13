package frc.robot.commands.macros.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class UpHigh extends CommandBase {

    private ClimberSubsystem climber;

    public UpHigh (ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(this.climber);
    }

    @Override
    public void execute() {
        climber.setMotorPosition(188_000.0, true);
    }
    
}
