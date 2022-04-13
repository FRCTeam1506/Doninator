package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoShootAndIndex extends CommandBase {

    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private TurretSubsystem turret;

    private boolean isAuto;
    private double velocity_rpm;
    
    public AutoShootAndIndex (ShooterSubsystem shooter, IndexerSubsystem indexer, TurretSubsystem turret) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.turret = turret;
        addRequirements(this.shooter, this.indexer, this.turret);

        this.isAuto = true;
    }

    public AutoShootAndIndex (ShooterSubsystem shooter, IndexerSubsystem indexer, TurretSubsystem turret, double velocity_rpm) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.turret = turret;
        addRequirements(this.shooter, this.indexer, this.turret);

        this.isAuto = false;
        this.velocity_rpm = velocity_rpm;
    } 

    @Override
    public void execute() {
        indexer.enableIndexing();

        double rpm;
        if (isAuto) {
            rpm = turret.calculateShooterRPM();
        } else {
            rpm = this.velocity_rpm;
        }

        shooter.shoot(rpm);
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
