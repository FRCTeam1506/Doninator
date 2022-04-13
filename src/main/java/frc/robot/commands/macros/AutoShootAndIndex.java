package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoShootAndIndex extends CommandBase {

    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private TurretSubsystem turret;
    
    public AutoShootAndIndex (ShooterSubsystem shooter, IndexerSubsystem indexer, TurretSubsystem turret) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.turret = turret;
        addRequirements(this.shooter, this.indexer, this.turret);
    }

    @Override
    public void execute() {
        indexer.enableIndexing();
        double calculatedRPM = turret.calculateShooterRPM();
        shooter.shoot(calculatedRPM);
        if (shooter.isShooterReady()) {
            indexer.enableShooting();
        }
    }

    @Override
    public void end (boolean interrupted) {
        indexer.disableIndexing();
        indexer.disableShooting();
    }

}
