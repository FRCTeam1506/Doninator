package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class StopClimber extends CommandBase {
    
    private ClimberSubsystem climber;

    public StopClimber (ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(this.climber);
    }

    @Override
    public void initialize() {
        climber.stopMotors();
    }
    
}
