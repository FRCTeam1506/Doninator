package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class AimAndControlTurret extends CommandBase {

    private static final double MAX_POWER = 0.15; // 0.20

    private TurretSubsystem turret;
    private SimpleMotorFeedforward feedforward;
    private PIDController controller;

    private DoubleSupplier powerSupplier;

    public AimAndControlTurret (TurretSubsystem turret, DoubleSupplier powerSupplier) {
        this.turret = turret;
        addRequirements(this.turret);

        // this.feedforward = new SimpleMotorFeedforward(1.0/360.0, 12.0/4098*1.92/60);

        this.controller = new PIDController(0.1, 0.001, 0.004); // 0.04
        this.controller.setTolerance(5.0); // 0.5

        this.powerSupplier = powerSupplier;
    }

    @Override
    public void initialize() {
        controller.setSetpoint(-0.3);
    }

    @Override
    public void execute() {
        if (turret.isAuto()) {
            if (turret.isTracking()) {
                double error = turret.getXError();
                // double output = feedforward.calculate(turret.getVelocity()) + controller.calculate(error);
                double output = controller.calculate(error);
                double power = MathUtil.clamp(output, -MAX_POWER, MAX_POWER);
                turret.setPower(power);
            } else {
                turret.setPosition(0.0);
            }
        } else {
            turret.setPower(powerSupplier.getAsDouble() * 0.1);
        }
    }
    
}
