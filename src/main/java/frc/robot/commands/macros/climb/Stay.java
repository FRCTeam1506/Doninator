package frc.robot.commands.macros.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class Stay extends CommandBase {

    private ClimberSubsystem climber;

    public Stay (ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(this.climber);
    }

    @Override
    public void execute() {
        climber.stopMotors();
    }

}
