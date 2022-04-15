package frc.robot.commands.macros.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class PullDown2 extends CommandBase {

    private static final double SETPOINT = -1_500.0;
    private static final double DELTA = 50.0;

    private ClimberSubsystem climber;

    public PullDown2 (ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(this.climber);
    }

    @Override
    public void execute() {
        climber.setMotorPosition(SETPOINT, false);
    }

    @Override
    public boolean isFinished() {
        double leftMotorPosition = climber.getLeftMotorPosition();
        double rightMotorPosition = climber.getRightMotorPosition();
        
        if (
            leftMotorPosition >= SETPOINT - DELTA && 
            leftMotorPosition <= SETPOINT + DELTA &&
            rightMotorPosition >= SETPOINT - DELTA &&
            rightMotorPosition <= SETPOINT + DELTA
        ) return true;
        return false;
    }
    
}
