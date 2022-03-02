package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAndIndex extends CommandBase {

    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private double velocity_rpm;
    
    public ShootAndIndex (ShooterSubsystem shooter, IndexerSubsystem indexer, double velocity_rpm) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.velocity_rpm = velocity_rpm;
        addRequirements(this.shooter, this.indexer);
    }

    @Override
    public void initialize() {
        indexer.enableIndexing();
    }

    @Override
    public void execute() {
        indexer.enableIndexing();
        shooter.shoot(velocity_rpm);
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
