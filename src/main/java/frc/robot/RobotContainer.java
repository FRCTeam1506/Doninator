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
import frc.robot.commands.autons.FourBall1;
import frc.robot.commands.autons.TwoBall1;
import frc.robot.commands.autons.TwoBallB1;
import frc.robot.commands.autons.TwoBallB2;
import frc.robot.commands.autons.TwoBallB3;
import frc.robot.commands.autons.TwoBallR1;
import frc.robot.commands.autons.TwoBallR2;
import frc.robot.commands.autons.TwoBallR3;
import frc.robot.commands.climber.ControlClimberStepper;
import frc.robot.commands.climber.ControlLeanboiMotors;
import frc.robot.commands.climber.MoveClimberDown;
import frc.robot.commands.climber.MoveClimberUp;
import frc.robot.commands.climber.ProgressClimber;
import frc.robot.commands.climber.RegressClimber;
import frc.robot.commands.climber.StopClimber;
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
import frc.robot.commands.shooter.StopShooter;
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
  public static Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

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
  private final Command c_runShooter = new ShootAndIndex(shooter, indexer, 1760.0); // 700.0 1850.0 1770.0 550.0 1970.0
  private final Command c_runShooterLow = new ShootAndIndex(shooter, indexer, 760.0);
  private final Command c_extendAndOuttake = new ExtendAndOuttake(intake, indexer);

  /* Trajectories */
  private PathPlannerTrajectory tr_test_1, tr_test_2,
    tr_two_ball_r1, tr_two_ball_r2, tr_two_ball_r3,
    tr_two_ball_b1, tr_two_ball_b2, tr_two_ball_b3,
    tr_four_ball_r1_1, tr_four_ball_r1_2, tr_four_ball_r1_3;

  private enum Autons { Nothing, TwoBall_R1, TwoBall_R2, TwoBall_R3, TwoBall_B1, TwoBall_B2, TwoBall_B3 }
  private SendableChooser<Autons> autonChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LiveWindow.disableAllTelemetry();
    DriverStation.silenceJoystickConnectionWarning(true);

    hub.enableCompressorAnalog(100, 120);
    // compressor.enableAnalog(100, 120);

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

    shooter.setDefaultCommand(c_idleShooter);

    intake.setDefaultCommand(c_stopAndRetract);

    indexer.setDefaultCommand(c_stopIndexer);

    turret.setDefaultCommand(c_aimTurret);

    climber.setDefaultCommand(c_controlClimbMotors); // c_stopClimber
  }

  private void configureButtonBindings () {
    zeroGyro.whenPressed(c_zeroGyro);
    shootAndIndex.whileHeld(c_runShooter);
    intakeAndIndex.whileHeld(c_runIndexer);
    extendAndOutake.whileHeld(c_extendAndOuttake);
    progressClimb.whenPressed(c_progressClimb);
    runClimbStep.whenPressed(
      new ControlClimberStepper(turret, climber, shooter)
    );
    toggleTurretControl.whenPressed(c_toggleTurretControl);

    new POVButton(operator, 0).whileHeld(c_runShooterLow);
    new POVButton(operator, 180).whenPressed(new RegressClimber(climber));
    new POVButton(operator, 90).whenPressed(new StopShooter(shooter).perpetually());
    new POVButton(operator, 270).whenPressed(new IdleShooter(shooter).perpetually());

    new POVButton(driver, 0).whenPressed(new InstantCommand(() -> hub.enableCompressorAnalog(100, 120)));
  }

  private void loadTrajectories () {
    tr_test_1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("test_1", 1.0, 1.0);
    tr_test_2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("test_2", 1.0, 1.0);

    tr_two_ball_r1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball_r1", 1.0, 1.0);
    tr_two_ball_r2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball_r2", 1.0, 1.0);
    tr_two_ball_r3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball_r3", 1.0, 1.0);

    tr_two_ball_b1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball_b1", 1.0, 1.0);
    tr_two_ball_b2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball_b2", 1.0, 1.0);
    tr_two_ball_b3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball_b3", 1.0, 1.0);

    tr_four_ball_r1_1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("four_ball_r1_1", 1.0, 1.0);
    tr_four_ball_r1_2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("four_ball_r1_2", 1.0, 1.0);
    tr_four_ball_r1_3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("four_ball_r1_3", 1.0, 1.0);
  }

  private void configureAuton () {
    autonChooser.setDefaultOption("Nothing", Autons.Nothing);
    autonChooser.addOption("2Ball R1", Autons.TwoBall_R1);
    autonChooser.addOption("2Ball R2", Autons.TwoBall_R2);
    autonChooser.addOption("2Ball R3", Autons.TwoBall_R3);
    autonChooser.addOption("2Ball B1", Autons.TwoBall_B1);
    autonChooser.addOption("2Ball B2", Autons.TwoBall_B2);
    autonChooser.addOption("2Ball B3", Autons.TwoBall_B3);

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

      case TwoBall_R1:
        return new TwoBallR1(drivetrain, intake, indexer, shooter, tr_two_ball_r1);

      case TwoBall_R2:
        return new TwoBallR2(drivetrain, intake, indexer, shooter, tr_two_ball_r2);

      case TwoBall_R3:
        return new TwoBallR3(drivetrain, intake, indexer, shooter, tr_two_ball_r3);

      case TwoBall_B1:
        return new TwoBallB1(drivetrain, intake, indexer, shooter, tr_two_ball_b1);

      case TwoBall_B2:
        return new TwoBallB2(drivetrain, intake, indexer, shooter, tr_two_ball_b2);

      case TwoBall_B3:
        return new TwoBallB3(drivetrain, intake, indexer, shooter, tr_two_ball_b3);
    
      default:
        return new WaitCommand(15.0);
    }
  }
}
