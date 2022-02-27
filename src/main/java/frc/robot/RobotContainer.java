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
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.climber.ProgressClimber;
import frc.robot.commands.climber.StopClimber;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory;
import frc.robot.commands.drivetrain.SwerveTeleop;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.indexer.StopIndexer;
import frc.robot.commands.intake.ExtendAndIntake;
import frc.robot.commands.intake.ExtendAndOuttake;
import frc.robot.commands.intake.StopAndRetract;
import frc.robot.commands.macros.IntakeAndIndex;
import frc.robot.commands.macros.ShootAndIndex;
import frc.robot.commands.shooter.IdleShooter;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.turret.AimTurret;
import frc.robot.commands.turret.RunTurret;
import frc.robot.commands.turret.StopTurret;
import frc.robot.commands.turret.ToggleTurretControlState;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.TrajectoryHelper;

public class RobotContainer {

  public static PneumaticHub hub = new PneumaticHub();
  // private Compressor compressor = hub.makeCompressor();
  // private Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0);
  private final PS4Controller operator = new PS4Controller(1);

  /* Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
  private final JoystickButton intakeAndIndex = new JoystickButton(driver, PS4Controller.Button.kL1.value);
  private final JoystickButton extendAndOuttake = new JoystickButton(driver, PS4Controller.Button.kSquare.value);

  private final JoystickButton toggleTurretControl = new JoystickButton(operator, PS4Controller.Button.kL1.value);
  private final JoystickButton shootAndIndex = new JoystickButton(operator, PS4Controller.Button.kR1.value);
  private final JoystickButton progressClimb = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);

  /* Subsystems */
  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  private final ShooterSubsystem shooter = new ShooterSubsystem(hub);
  private final IntakeSubsystem intake = new IntakeSubsystem(hub);
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final TurretSubsystem turret = new TurretSubsystem(hub);
  private final ClimberSubsystem climber = new ClimberSubsystem(hub);

  /* Commands */
  // * primitives
  private final Command c_zeroGyro = new InstantCommand( () -> drivetrain.zeroGyro() );
  private final Command c_extendAndOuttake = new ExtendAndOuttake(intake);
  private final Command c_stopAndRetract = new StopAndRetract(intake);
  private final Command c_idleShooter = new IdleShooter(shooter);
  private final Command c_stopIndexer = new StopIndexer(indexer);
  private final Command c_stopClimber = new StopClimber(climber);
  private final Command c_progressClimb = new ProgressClimber(climber);
  private final Command c_aimTurret = new AimTurret(turret, () -> operator.getLeftX());
  private final Command c_toggleTurretControl = new ToggleTurretControlState(turret);

  // // * macros
  private final Command c_runIndexer = new IntakeAndIndex(intake, indexer);
  private final Command c_runShooter = new ShootAndIndex(shooter, indexer, 550.0); // 700.0 1850.0 1770.0

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

    shooter.setDefaultCommand(c_idleShooter);

    intake.setDefaultCommand(c_stopAndRetract);

    indexer.setDefaultCommand(c_stopIndexer);

    turret.setDefaultCommand(c_aimTurret);

    climber.setDefaultCommand(c_stopClimber);
  }

  private void configureButtonBindings () {
    zeroGyro.whenPressed(c_zeroGyro);
    shootAndIndex.whileHeld(c_runShooter);
    intakeAndIndex.whileHeld(c_runIndexer);
    extendAndOuttake.whileHeld(c_extendAndOuttake);
    progressClimb.whenPressed(c_progressClimb);
    toggleTurretControl.whenPressed(c_toggleTurretControl);

    // new POVButton(driver, 0).whenPressed(new RunTurret(turret, 0));
    // new POVButton(driver, 90).whenPressed(new RunTurret(turret, 5000.0));
    // new POVButton(driver, 270).whenPressed(new RunTurret(turret, 8000.0));
    // new POVButton(driver, 180).whenPressed(new RunTurret(turret, -8000.0));
  }

  private void loadTrajectories() {
    tr_test_1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("test_1");
  }

  public Command getAutonomousCommand() {
    // return new RunPathPlannerTrajectory2(drivetrain, tr_test_1);
    // return null;
    return new WaitCommand(1.0);
  }
}
