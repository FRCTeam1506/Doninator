package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.TurretSubsystem;

public class AimAndControlTurret2 extends PIDCommand {

    private static final double MAX_POWER = 0.27;

    public AimAndControlTurret2 (TurretSubsystem turret, DoubleSupplier powerSupplier) {
        super(
            new PIDController(0.03, 0.001, 0.0),
            turret::getXError,
            -0.35,
            output -> {
                if (turret.isAuto()) {
                    if (turret.isTracking()) {
                        // System.out.println(output);
                        double m_power = MathUtil.clamp(output, -MAX_POWER, MAX_POWER);
                        turret.setPower(m_power);
                    } else {
                        turret.setPosition(0.0);
                    }
                } else {
                    turret.setPower(powerSupplier.getAsDouble() * 0.1);
                }
            },
            turret
        );

        getController().setTolerance(0.05);
    }

}
