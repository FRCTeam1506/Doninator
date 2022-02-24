package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendAndOuttake extends CommandBase {

    private IntakeSubsystem intake;

    public ExtendAndOuttake (IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        intake.extend();
        intake.outtake();
    }
    
}
