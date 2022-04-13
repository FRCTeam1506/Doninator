package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class SetPosition extends CommandBase {

    private ClimberSubsystem climber;
    private double position;

    public SetPosition (ClimberSubsystem climber, double position) {
        this.climber = climber;
        addRequirements(this.climber);

        this.position = position;
    }

    @Override
    public void execute() {
        climber.setMotorPosition(position);
    }

    @Override
    public boolean isFinished() {
        if (climber.getMotorPosition() >= Math.abs(position) - 100.0 && climber.getMotorPosition() <= Math.abs(position) + 100.0) return true;
        return false;
    }
    
}
