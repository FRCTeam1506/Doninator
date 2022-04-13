package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class Retract extends CommandBase {

    private IntakeSubsystem intake;

    public Retract (IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        intake.retract();
        intake.stop();
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
    
}
