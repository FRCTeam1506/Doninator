package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.macros.climb.Step1;
import frc.robot.commands.macros.climb.Step10;
import frc.robot.commands.macros.climb.Step2;
import frc.robot.commands.macros.climb.Step3;
import frc.robot.commands.macros.climb.Step4;
import frc.robot.commands.macros.climb.Step5;
import frc.robot.commands.macros.climb.Step6;
import frc.robot.commands.macros.climb.Step7;
import frc.robot.commands.macros.climb.Step8;
import frc.robot.commands.macros.climb.Step9;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ControlClimberStepper extends SequentialCommandGroup {

    private TurretSubsystem turret;
    private ClimberSubsystem climber;
    private ShooterSubsystem shooter;

    public ControlClimberStepper (TurretSubsystem turret, ClimberSubsystem climber, ShooterSubsystem shooter) {
        this.turret = turret;
        this.climber = climber;
        this.shooter = shooter;
        addRequirements(this.turret, this.climber, this.shooter);
    }

    @Override
    public void execute() {
        int currentState = climber.currentClimbState;
        switch (currentState) {
            case 1:
                CommandScheduler.getInstance().schedule(new Step1(turret, climber, shooter));
                break;

            case 2:
                CommandScheduler.getInstance().schedule(new Step2(climber));
                break;

            case 3:
                CommandScheduler.getInstance().schedule(new Step3(climber));
                break;

            case 4:
                CommandScheduler.getInstance().schedule(new Step4(climber));
                break;

            case 5:
                CommandScheduler.getInstance().schedule(new Step5(climber));
                break;

            case 6:
                CommandScheduler.getInstance().schedule(new Step6(climber));
                break;

            case 7:
                CommandScheduler.getInstance().schedule(new Step7(climber));
                break;

            case 8:
                CommandScheduler.getInstance().schedule(new Step8(climber));
                break;

            case 9:
                CommandScheduler.getInstance().schedule(new Step10(climber));
                break;

            case 10:
                CommandScheduler.getInstance().schedule(new Step4(climber));
                break;

            case 11:
                CommandScheduler.getInstance().schedule(new Step5(climber));
                break;

            case 12:
                CommandScheduler.getInstance().schedule(new Step6(climber));
                break;

            case 13:
                CommandScheduler.getInstance().schedule(new Step7(climber));
                break;

            case 14:
                CommandScheduler.getInstance().schedule(new Step8(climber));
                break;

            case 15:
                CommandScheduler.getInstance().schedule(new Step9(climber));
                break;
        
            default:
                break;
        }

        // climber.currentClimbState++;
    }

}
