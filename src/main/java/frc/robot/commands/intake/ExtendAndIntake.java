package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendAndIntake extends CommandBase {

    private IntakeSubsystem intake;
    
    public ExtendAndIntake (IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        intake.extend();
        intake.intake();
    }

}
