package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class AimTurret extends PIDCommand {

    public AimTurret (TurretSubsystem turret, DoubleSupplier power) {
        super(
            new PIDController(
                Constants.Turret.AUTO_kP, 
                Constants.Turret.AUTO_kI,
                Constants.Turret.AUTO_kD
            ),
            turret::getXError, 
            -2.0,
            output -> {
                if (turret.isAuto()) {
                    if (turret.isTracking()) {
                        System.out.println(output);
                        double m_power = MathUtil.clamp(output, -0.2, 0.2);
                        turret.setPower(m_power);
                    } else {
                        turret.setPosition(0.0);
                    }
                } else {
                    System.out.println(power.getAsDouble() * 0.1);
                    turret.setPower(power.getAsDouble() * 0.1);
                }
            },
            turret
        );
        getController().setTolerance(1.0); // 0.2
    }

    // @Override
    // public boolean isFinished() { return getController().atSetpoint(); }

}
