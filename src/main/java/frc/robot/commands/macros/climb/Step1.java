package frc.robot.commands.macros.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class Step1 extends CommandBase {

    private TurretSubsystem turret;
    private ClimberSubsystem climber;
    private ShooterSubsystem shooter;

    public Step1 (TurretSubsystem turret, ClimberSubsystem climber, ShooterSubsystem shooter) {
        this.turret = turret;
        this.climber = climber;
        this.shooter = shooter;
        addRequirements(this.turret, this.climber);
    }

    @Override
    public void execute() {
        // shooter.shoot(0.0);
        shooter.setDefaultCommand(
            new StopShooter(shooter)
        );

        // RobotContainer.compressor.disable();

        turret.setPosition(0.0);
        turret.putHoodDown();

        climber.extendExtendo();
        climber.retractLeanboi();
        climber.retractTrigger();
    }
    
}
