package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class IdleShooter extends CommandBase {

    private ShooterSubsystem shooter;

    private static final double IDLE_VELOCITY_RPM = 420.69;

    public IdleShooter (ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.shoot(IDLE_VELOCITY_RPM);
    }
}
