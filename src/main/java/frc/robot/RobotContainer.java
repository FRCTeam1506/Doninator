// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.primitives.SwerveTrajectory;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.TrajectoryHelper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */
  private final Joystick driver = new Joystick(0);

  /* Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, Constants.Playstation.CircleButton);

  /* Subsystems */
  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  /* Commands */

  /* Trajectories */
  private Trajectory tr_test;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    setDefaultCommands();
    loadTrajectories();
  }


  private void configureButtonBindings() {
    zeroGyro.whenPressed(new InstantCommand( () -> drivetrain.zeroGyro() ));
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
      new TeleopSwerve(
        drivetrain,
        driver,
        true, true
      )
    );
  }

  private void loadTrajectories() {
    tr_test = TrajectoryHelper.loadTrajectoryFromFile("test1");
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SwerveTrajectory(drivetrain, tr_test);
  }
}
