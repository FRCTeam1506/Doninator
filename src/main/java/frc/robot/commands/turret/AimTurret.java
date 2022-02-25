package frc.robot.commands.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class AimTurret extends PIDCommand {

    public AimTurret (TurretSubsystem turret) {
        super(
            new PIDController(
                Constants.Turret.AUTO_kP, 
                Constants.Turret.AUTO_kI,
                Constants.Turret.AUTO_kD
            ),
            turret::getSensorPosition, 
            2.0,
            output -> { turret.setPosition(output); },
            turret
        );

        getController().setTolerance(0.2);
    }

    @Override
    public boolean isFinished() { return getController().atSetpoint(); }

}
