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
            turret::getXError, 
            -2.0,
            output -> {
                System.out.println(output);
                if (Math.abs(output) > 0.0 && Math.abs(output) < 0.5) {
                    turret.setPower(-output);
                } else {
                    if (output > 0.0) { // positive
                        turret.setPower(-0.5);
                    } else {
                        turret.setPower(0.5);
                    }
                }
            },
            turret
        );
        getController().setTolerance(0.2);
    }

    @Override
    public boolean isFinished() { return getController().atSetpoint(); }

}
