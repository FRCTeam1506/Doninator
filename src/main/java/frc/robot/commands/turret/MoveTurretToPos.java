package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class MoveTurretToPos extends CommandBase {

    private TurretSubsystem turret;
    private double position;

    public MoveTurretToPos (TurretSubsystem turret, double position) {
        this.turret = turret;
        this.position = position;
        addRequirements(this.turret);
    }

    @Override
    public void initialize() {
        turret.setPosition(position);
    }

}
