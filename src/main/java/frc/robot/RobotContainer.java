// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.drivetrain.SwerveTeleop;
import frc.robot.commands.shooter.IdleShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.TrajectoryHelper;

public class RobotContainer {

  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0);

  /* Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
  // private final JoystickButton shoot = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);

  /* Subsystems */
  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  // private final ShooterSubsystem shooter = new ShooterSubsystem();

  /* Commands */
  private final Command c_zeroGyro = new InstantCommand( () -> drivetrain.zeroGyro() );
  // private final Command c_shoot = new Shoot(shooter, 1850.0);

  /* Trajectories */
  private PathPlannerTrajectory tr_test_1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LiveWindow.disableAllTelemetry();
    DriverStation.silenceJoystickConnectionWarning(true);

    setDefaultCommands();
    configureButtonBindings();
    loadTrajectories();
  }

  private void configureButtonBindings() {
    zeroGyro.whenPressed(c_zeroGyro);
    // new JoystickButton(driver, PS4Controller.Button.kTriangle.value).whileHeld(c_shoot);
    // shoot.whileHeld(c_shoot);
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
      new SwerveTeleop(
        drivetrain,
        driver,
        true, true
      )
    );

    // shooter.setDefaultCommand(new IdleShooter(shooter));
  }

  private void loadTrajectories() {
    // tr_test = TrajectoryHelper.loadWPILibTrajectoryFromFile("test1");
    // tr_straight = TrajectoryHelper.loadWPILibTrajectoryFromFile("straight");
    // tr_holotest = TrajectoryHelper.loadPathPlannerTrajectory("straight2");
    tr_test_1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("test_1");
  }

  public Command getAutonomousCommand() {
    return new RunPathPlannerTrajectory2(drivetrain, tr_test_1);
  }
}
