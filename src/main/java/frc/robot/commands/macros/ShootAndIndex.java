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
        // double calculatedRPM = turret.calculateShooterRPM();
        // if (calculatedRPM > 0 && calculatedRPM < 2200.0) {

        // } else {
        //     shooter.shoot();
        // }
        shooter.shoot(this.velocity_rpm);
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
