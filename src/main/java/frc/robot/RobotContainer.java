// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.SwerveTeleop;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.TrajectoryHelper;


public class RobotContainer {
  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0);

  /* Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kCircle.value);

  /* Subsystems */
  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  /* Commands */

  /* Trajectories */
  private Trajectory tr_test, tr_straight, tr_holotest;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    LiveWindow.disableAllTelemetry();
    DriverStation.silenceJoystickConnectionWarning(true);

    setDefaultCommands();
    configureButtonBindings();
    loadTrajectories();
  }

  private void configureButtonBindings() {
    zeroGyro.whenPressed(new InstantCommand( () -> drivetrain.zeroGyro() ));
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
      new SwerveTeleop(
        drivetrain,
        driver,
        true, true
      )
    );
  }

  private void loadTrajectories() {
    tr_test = TrajectoryHelper.loadWPILibTrajectoryFromFile("test1");
    tr_straight = TrajectoryHelper.loadWPILibTrajectoryFromFile("straight");
    tr_holotest = TrajectoryHelper.loadPathPlannerTrajectory("straight2");
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
