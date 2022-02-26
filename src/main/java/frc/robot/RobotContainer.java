// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory;
import frc.robot.commands.drivetrain.SwerveTeleop;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.TrajectoryHelper;

public class RobotContainer {

  public static PneumaticHub hub = new PneumaticHub();
  // private Compressor compressor = hub.makeCompressor();
  // private Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0);
  // private final PS4Controller operator = new PS4Controller(1);

  /* Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kCircle.value);

  /* Subsystems */
  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  /* Commands */
  // * primitives
  private final Command c_zeroGyro = new InstantCommand( () -> drivetrain.zeroGyro() );

  /* Trajectories */
  private PathPlannerTrajectory tr_test_1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LiveWindow.disableAllTelemetry();
    DriverStation.silenceJoystickConnectionWarning(true);

    hub.enableCompressorAnalog(100, 120);
    // compressor.enableAnalog(100, 120);

    setDefaultCommands();
    configureButtonBindings();
    loadTrajectories();
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

  private void configureButtonBindings () {
    zeroGyro.whenPressed(c_zeroGyro);
  }

  private void loadTrajectories() {
    tr_test_1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("test_1");
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(1.0);
    // return new RunPathPlannerTrajectory2(drivetrain, tr_test_1);
    // return null;
  }
}
