package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootAndIndex extends CommandBase {

    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private TurretSubsystem turret;
    private double velocity_rpm;
    
    public ShootAndIndex (ShooterSubsystem shooter, IndexerSubsystem indexer, TurretSubsystem turret, double velocity_rpm) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.turret = turret;
        this.velocity_rpm = velocity_rpm;
        addRequirements(this.shooter, this.indexer, this.turret);
    }

    @Override
    public void initialize() {
        indexer.enableIndexing();
    }

    @Override
    public void execute() {
        indexer.enableIndexing();
        shooter.shoot(turret.calculateShooterRPM());
        System.out.println("AAAAAAAAAAAAAAAAAAAAA");
        if (shooter.isShooterReady()) {
            System.out.println("BBBBBBBBBBBBBBBBBBBBB");
            indexer.enableShooting();
        }
    }

    @Override
    public void end (boolean interrupted) {
        indexer.disableIndexing();
        indexer.disableShooting();
    }

}
