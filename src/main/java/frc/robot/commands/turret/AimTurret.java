package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

public class AimTurret extends PIDCommand {

    private static final double MAX_POWER = 0.27;

    public AimTurret (TurretSubsystem turret, DoubleSupplier power) {
        super(
            new PIDController(
                Constants.Turret.AUTO_kP, 
                Constants.Turret.AUTO_kI,
                Constants.Turret.AUTO_kD
            ),
            turret::getXError, 
            -1.0, // -2.0
            output -> {
                if (turret.isAuto()) {
                    if (turret.isTracking()) {
                        System.out.println(output);
                        double m_power = MathUtil.clamp(output, -MAX_POWER, MAX_POWER);
                        turret.setPower(-m_power);
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
        getController().setTolerance(0.07); // 0.2
    }

    // @Override
    // public boolean isFinished() { return getController().atSetpoint(); }

}
