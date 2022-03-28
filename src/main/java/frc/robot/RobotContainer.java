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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory;
import frc.robot.commands.drivetrain.SwerveTeleop;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.TrajectoryHelper;

public class RobotContainer {

  public static PneumaticHub hub = new PneumaticHub();
  public static Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0);

  /* Buttons */
  // * driver
  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kCircle.value);

  /* Subsystems */
  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();

  /* Commands */
  // * primitives
  private final Command c_zeroGyro = new InstantCommand( () -> drivetrain.zeroGyro() );

  /* Trajectories */
  private PathPlannerTrajectory tr_straight_x, tr_straight_y, tr_straight_rot, 
                                tr_small_curve, tr_small_curve_rot, 
                                tr_fancy_1, tr_fancy_2;

  private enum Autons { Nothing, StraightX, StraightY, StraightCurve, SmallCurve, SmallCurveRot, Fancy1, Fancy2 }
  private SendableChooser<Autons> autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LiveWindow.disableAllTelemetry();
    DriverStation.silenceJoystickConnectionWarning(true);

    hub.enableCompressorAnalog(100, 120);

    setDefaultCommands();
    configureButtonBindings();
    loadTrajectories();
    configureAuton();
    dashboardStuff();
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
    new POVButton(driver, 0).whenPressed(new InstantCommand(() -> hub.enableCompressorAnalog(100, 120)));
    
    new POVButton(driver, 270).whenPressed(new DriveDistance(drivetrain, 2.0, true));
    new POVButton(driver, 90).whenPressed(new DriveDistance(drivetrain, 2.0, false));
  }

  private void loadTrajectories () {
    tr_straight_x = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("straight_x", 1.5, 1.0);
    tr_straight_y = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("straight_y", 1.5, 1.0);
    tr_straight_rot = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("straight_rot");

    tr_small_curve = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("small_curve", 2.3, 1.5);
    // tr_small_curve_rot = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("small_curve_rot");

    // tr_fancy_1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("fancy_1");
    // tr_fancy_2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("fancy_2");
  }

  private void configureAuton () {
    autonChooser.setDefaultOption("Nothing", Autons.Nothing);
    autonChooser.addOption(Autons.StraightX.name(), Autons.StraightX);
    autonChooser.addOption(Autons.StraightY.name(), Autons.StraightY);
    autonChooser.addOption(Autons.StraightCurve.name(), Autons.StraightCurve);
    autonChooser.addOption(Autons.SmallCurve.name(), Autons.SmallCurve);
    autonChooser.addOption(Autons.SmallCurveRot.name(), Autons.SmallCurveRot);
    autonChooser.addOption(Autons.Fancy1.name(), Autons.Fancy1);
    autonChooser.addOption(Autons.Fancy2.name(), Autons.Fancy2);

    ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    tab.add(autonChooser);
  }

  private void dashboardStuff () {
    ShuffleboardTab tab = Shuffleboard.getTab("Climber");
    tab.addNumber("PSI", () -> hub.getPressure(0));
  }

  public Command getAutonomousCommand () {
    switch (autonChooser.getSelected()) {
      case Nothing:
        return new WaitCommand(15.0);

      case StraightX:
        return new RunPathPlannerTrajectory(drivetrain, tr_straight_x);

      case StraightY:
        return new RunPathPlannerTrajectory(drivetrain, tr_straight_y);

      case StraightCurve:
        return new RunPathPlannerTrajectory(drivetrain, tr_straight_rot);

      case SmallCurve:
        return new RunPathPlannerTrajectory(drivetrain, tr_small_curve);

      case SmallCurveRot:
        return new RunPathPlannerTrajectory(drivetrain, tr_small_curve_rot);

      case Fancy1:
        return new RunPathPlannerTrajectory(drivetrain, tr_fancy_1);

      case Fancy2:
        return new RunPathPlannerTrajectory(drivetrain, tr_fancy_2);
    
      default:
        return new WaitCommand(15.0);
    }
  }
}
