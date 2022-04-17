package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.macros.climb.HangTraversal;
import frc.robot.commands.macros.climb.HangTraversal2;
import frc.robot.commands.macros.climb.Stay;
import frc.robot.commands.macros.climb.UpLittle;
import frc.robot.commands.macros.climb.UpLittle2;
import frc.robot.commands.macros.climb.RetractLeanboi;
import frc.robot.commands.macros.climb.StartClimb;
import frc.robot.commands.macros.climb.UpHigh;
import frc.robot.commands.macros.climb.UpHigh2;
import frc.robot.commands.macros.climb.ExtendLeanboi;
import frc.robot.commands.macros.climb.PullDown;
import frc.robot.commands.macros.climb.PullDown2;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RunClimbStep2 extends SequentialCommandGroup {

    private TurretSubsystem turret;
    private ClimberSubsystem climber;
    private ShooterSubsystem shooter;

    public RunClimbStep2 (TurretSubsystem turret, ClimberSubsystem climber, ShooterSubsystem shooter) {
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
                CommandScheduler.getInstance().schedule(new StartClimb(turret, climber, shooter));
                break;

            case 2:
                CommandScheduler.getInstance().schedule(new PullDown(climber));
                break;

            case 3:
                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new UpLittle2(climber),
                    new RetractLeanboi(climber).withTimeout(0.3),
                    new UpHigh2(climber)
                ));
                break;

            case 4:
                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new ExtendLeanboi(climber).withTimeout(0.5),
                    new PullDown2(climber)
                ));
                break;
                
            case 5:
                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new UpLittle2(climber),
                    new RetractLeanboi(climber).withTimeout(0.3),
                    new UpHigh2(climber)
                ));
                break;

            case 6:
                CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                    new ExtendLeanboi(climber).withTimeout(0.5),
                    new HangTraversal2(climber),
                    new Stay(climber)
                ));
                break;
        
            default:
                break;
        }

        // climber.currentClimbState++;
    }

}
