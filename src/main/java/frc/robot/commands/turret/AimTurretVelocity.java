
package frc.robot.commands.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class AimTurretVelocity extends CommandBase {

    private static final double MAX_VELOCITY = 500.0; // 2_000.0

    private TurretSubsystem turret;
    private PIDController controller;
    // private SimpleMotorFeedforward feedforward;

    public AimTurretVelocity (TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(this.turret);

        controller = new PIDController(1.0, 0.0, 0.0);
        controller.setTolerance(0.1);
        // feedforward = new SimpleMotorFeedforward(ks, kv);
    }

    @Override
    public void initialize() {
        controller.setSetpoint(-1.0);
    }

    @Override
    public void execute() {
        double output = controller.calculate(turret.getXError());

        double velocity = MathUtil.clamp(output, -MAX_VELOCITY, MAX_VELOCITY);
        turret.setVelocity(-velocity);

        System.out.println("Output: " + output);
        System.out.println("Velocity: " + -velocity);
    }

    // @Override
    // public boolean isFinished() { return controller.atSetpoint(); }

}
