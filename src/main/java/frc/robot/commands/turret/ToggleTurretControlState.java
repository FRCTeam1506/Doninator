package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class ToggleTurretControlState extends CommandBase {
    
    private TurretSubsystem turret;

    public ToggleTurretControlState (TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(this.turret);
    }

    @Override
    public void initialize() {
        turret.toggleControlState();
    }

}
