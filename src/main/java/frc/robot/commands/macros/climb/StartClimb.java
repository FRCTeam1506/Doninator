package frc.robot.commands.macros.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.turret.StopTurret;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class StartClimb extends CommandBase {

    private TurretSubsystem turret;
    private ClimberSubsystem climber;
    private ShooterSubsystem shooter;

    public StartClimb (TurretSubsystem turret, ClimberSubsystem climber, ShooterSubsystem shooter) {
        this.turret = turret;
        this.climber = climber;
        this.shooter = shooter;
        addRequirements(this.turret, this.climber, this.shooter);
    }

    @Override
    public void execute() {
        shooter.setDefaultCommand(
            new StopShooter(shooter)
        );

        turret.setPosition(0.0);

        turret.setDefaultCommand(
            new StopTurret(turret)
        );

        climber.setMotorPosition(165_000.0, true);
    }
    
}
