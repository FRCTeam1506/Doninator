package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ControlLeanboiMotors extends CommandBase {
    
    private ClimberSubsystem climber;
    private DoubleSupplier leftPower, rightPower;

    public ControlLeanboiMotors (ClimberSubsystem climber, DoubleSupplier leftPower, DoubleSupplier rightPower) {
        this.climber = climber;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        addRequirements(this.climber);
    }

    @Override
    public void execute() {
        climber.setPower(leftPower.getAsDouble() * 0.07, rightPower.getAsDouble() * 0.07);
    }

}
