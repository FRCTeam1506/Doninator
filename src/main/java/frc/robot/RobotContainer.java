// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.autons.FourBall1;
import frc.robot.commands.autons.TwoBall1;
import frc.robot.commands.climber.ControlClimberStepper;
import frc.robot.commands.climber.ControlLeanboiMotors;
import frc.robot.commands.climber.MoveClimberDown;
import frc.robot.commands.climber.MoveClimberUp;
import frc.robot.commands.climber.ProgressClimber;
import frc.robot.commands.climber.StopClimber;
import frc.robot.commands.drivetrain.DriveDistance;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory;
import frc.robot.commands.drivetrain.SwerveTeleop;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.indexer.StopIndexer;
import frc.robot.commands.intake.ExtendAndIntake;
import frc.robot.commands.macros.ExtendAndOuttake;
import frc.robot.commands.intake.StopAndRetract;
import frc.robot.commands.macros.IntakeAndIndex;
import frc.robot.commands.macros.ShootAndIndex;
import frc.robot.commands.shooter.IdleShooter;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.turret.AimTurret;
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
  // * driver
  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
  // private final JoystickButton extendAndOuttake = new JoystickButton(driver, PS4Controller.Button.kSquare.value);

  // * operator
  private final JoystickButton intakeAndIndex = new JoystickButton(operator, PS4Controller.Button.kL1.value);
  private final JoystickButton toggleTurretControl = new JoystickButton(operator, PS4Controller.Button.kTriangle.value);
  private final JoystickButton shootAndIndex = new JoystickButton(operator, PS4Controller.Button.kR1.value);
  private final JoystickButton progressClimb = new JoystickButton(operator, PS4Controller.Button.kCircle.value);
  private final JoystickButton extendAndOutake = new JoystickButton(operator, PS4Controller.Button.kSquare.value);
  private final JoystickButton runClimbStep = new JoystickButton(operator, PS4Controller.Button.kCross.value);

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
  private final Command c_stopAndRetract = new StopAndRetract(intake);
  private final Command c_idleShooter = new IdleShooter(shooter);
  private final Command c_stopIndexer = new StopIndexer(indexer);
  private final Command c_stopClimber = new StopClimber(climber);
  private final Command c_progressClimb = new ProgressClimber(climber);
  private final Command c_aimTurret = new AimTurret(turret, () -> operator.getRightX());
  private final Command c_controlClimbMotors = new ControlLeanboiMotors(climber, () -> operator.getLeftY(), () -> operator.getRightY());
  private final Command c_toggleTurretControl = new ToggleTurretControlState(turret);

  // * macros
  private final Command c_runIndexer = new IntakeAndIndex(intake, indexer);
  private final Command c_runShooter = new ShootAndIndex(shooter, indexer, 1965.0); // 700.0 1850.0 1770.0 550.0

  /* Trajectories */
  private PathPlannerTrajectory tr_test_1, tr_two_ball, tr_two_ball_pos3, tr_four_ball_pos1_1, tr_four_ball_pos1_2, tr_four_ball_pos1_3;

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
    extendAndOutake.whileHeld(c_extendAndOuttake);
    progressClimb.whenPressed(c_progressClimb);
    runClimbStep.whileHeld(
      new ControlClimberStepper(turret, climber, shooter)
    );
    toggleTurretControl.whenPressed(c_toggleTurretControl);

    new POVButton(operator, 0).whileHeld(new MoveClimberUp(climber));
    new POVButton(operator, 180).whileHeld(new MoveClimberDown(climber));

    // new POVButton(operator, 0).whenPressed(new InstantCommand(() -> climber.extendLeanboi(), climber));
    // new POVButton(operator, 180).whenPressed(new InstantCommand(() -> climber.retractLeanboi(), climber));
    // new POVButton(operator, 270).whenPressed(new InstantCommand(() -> climber.retractTrigger(), climber));
    // new POVButton(operator, 90).whenPressed(new InstantCommand(() -> climber.extendTrigger(), climber));
    // new JoystickButton(operator, PS4Controller.Button.kTriangle.value).whenPressed(new InstantCommand(() -> climber.retractExtendo(), climber));
    // new JoystickButton(operator, PS4Controller.Button.kCross.value).whenPressed(new InstantCommand(() -> climber.extendExtendo(), climber));
  }

  private void loadTrajectories() {
    tr_test_1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("test_1");
    tr_two_ball = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball");
    tr_two_ball_pos3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball_pos3");
    tr_four_ball_pos1_1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("four_ball_pos1_1");
    tr_four_ball_pos1_2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("four_ball_pos1_2");
    tr_four_ball_pos1_3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("four_ball_pos1_3");
  }

  public Command getAutonomousCommand() {
    // return new RunPathPlannerTrajectory(drivetrain, tr_test_1);
    return new TwoBall1(drivetrain, intake, indexer, shooter, turret, tr_two_ball);
    // return new FourBall1(drivetrain, intake, indexer, shooter, turret, tr_four_ball_pos1_1, tr_four_ball_pos1_2, tr_four_ball_pos1_3);
    // return new DriveDistance(drivetrain, Units.inchesToMeters(20), true);
  }
}
