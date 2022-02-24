package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class RunTurret extends CommandBase {

    private TurretSubsystem turret;
    private double position;

    public RunTurret (TurretSubsystem turret, double position) {
        this.turret = turret;
        this.position = position;
        addRequirements(this.turret);
    }

    @Override
    public void initialize() {
        turret.setPosition(position);
    }

}
