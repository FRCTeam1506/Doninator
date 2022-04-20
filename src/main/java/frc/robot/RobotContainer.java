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
import frc.robot.commands.autons.FiveBallAR1;
import frc.robot.commands.autons.FiveBallR1;
import frc.robot.commands.autons.TwoBallB1;
import frc.robot.commands.autons.TwoBallB2;
import frc.robot.commands.autons.TwoBallB3;
import frc.robot.commands.autons.TwoBallR1;
import frc.robot.commands.autons.TwoBallR2;
import frc.robot.commands.autons.TwoBallR3;
import frc.robot.commands.climber.RunClimbStep;
import frc.robot.commands.climber.RunClimbStep2;
import frc.robot.commands.climber.SetPosition;
import frc.robot.commands.climber.ControlLeanboiMotors;
import frc.robot.commands.climber.ProgressClimbStep;
import frc.robot.commands.climber.RegressClimbStep;
import frc.robot.commands.climber.StopClimber;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory;
import frc.robot.commands.drivetrain.StopMoving;
import frc.robot.commands.drivetrain.SwerveTeleop;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.indexer.RunIndexerShooting;
import frc.robot.commands.indexer.StopIndexer;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.macros.AutoShootAndIndex;
import frc.robot.commands.macros.ExtendAndOuttake;
import frc.robot.commands.macros.IntakeAndIndex;
import frc.robot.commands.macros.ShootAndIndex;
import frc.robot.commands.shooter.IdleShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.turret.AimAndControlTurret;
import frc.robot.commands.turret.AimAndControlTurret2;
import frc.robot.commands.turret.AimTurret;
import frc.robot.commands.turret.ToggleTurretControlState;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LittyBoi;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.TrajectoryHelper;

public class RobotContainer {

  public static PneumaticHub hub      = new PneumaticHub();
  public static Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  /* Controllers */
  private final PS4Controller driver    = new PS4Controller(0);
  private final PS4Controller operator  = new PS4Controller(1);
  // private final PS4Controller superman  = new PS4Controller(2);

  /* Buttons */
  // * driver
  private final JoystickButton DR_circle      = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
  private final JoystickButton DR_leftBumper  = new JoystickButton(driver, PS4Controller.Button.kL1.value);
  private final POVButton DR_up = new POVButton(driver, 0);

  // * operator
  private final JoystickButton OP_leftBumper  = new JoystickButton(operator, PS4Controller.Button.kL1.value);
  private final JoystickButton OP_rightBumper = new JoystickButton(operator, PS4Controller.Button.kR1.value);
  private final JoystickButton OP_circle      = new JoystickButton(operator, PS4Controller.Button.kCircle.value);
  private final JoystickButton OP_triangle    = new JoystickButton(operator, PS4Controller.Button.kTriangle.value);
  private final JoystickButton OP_square      = new JoystickButton(operator, PS4Controller.Button.kSquare.value);
  private final JoystickButton OP_cross       = new JoystickButton(operator, PS4Controller.Button.kCross.value);
  private final JoystickButton OP_big         = new JoystickButton(operator, PS4Controller.Button.kTouchpad.value);
  private final JoystickButton OP_options     = new JoystickButton(operator, PS4Controller.Button.kOptions.value);

  private final POVButton OP_up     = new POVButton(operator, 0);
  private final POVButton OP_right  = new POVButton(operator, 90);
  private final POVButton OP_down   = new POVButton(operator, 180);
  private final POVButton OP_left   = new POVButton(operator, 270);

  // * superman
  // private final JoystickButton S_square   = new JoystickButton(superman, PS4Controller.Button.kSquare.value);
  // private final JoystickButton S_circle   = new JoystickButton(superman, PS4Controller.Button.kCircle.value);
  // private final JoystickButton S_triangle = new JoystickButton(superman, PS4Controller.Button.kTriangle.value);
  // private final JoystickButton S_cross    = new JoystickButton(superman, PS4Controller.Button.kCross.value);

  // private final POVButton S_up     = new POVButton(superman, 0);
  // private final POVButton S_right  = new POVButton(superman, 90);
  // private final POVButton S_down   = new POVButton(superman, 180);
  // private final POVButton S_left   = new POVButton(superman, 270);

  /* Subsystems */
  public final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  public final ShooterSubsystem shooter    = new ShooterSubsystem(hub);
  public final IntakeSubsystem intake      = new IntakeSubsystem(hub);
  public final IndexerSubsystem indexer    = new IndexerSubsystem();
  public final TurretSubsystem turret      = new TurretSubsystem(hub);
  public final ClimberSubsystem climber    = new ClimberSubsystem(hub);
  public final LittyBoi littyBoi           = new LittyBoi(this);

  /* Commands */
  // * primitives
  // ? Drivetrain
  private final Command c_zeroGyro = new InstantCommand( () -> drivetrain.zeroGyro() );
  private final Command c_stopMoving = new StopMoving(drivetrain);

  // ? Intake
  private final Command c_retract = new Retract(intake);

  // ? Shooter
  private final Command c_idleShooter = new IdleShooter(shooter);

  // ? Indexer
  private final Command c_stopIndexer = new StopIndexer(indexer);
  private final Command c_runIndexer = new RunIndexerShooting(indexer);

  // ? Climber
  private final Command c_progressClimbStep   = new ProgressClimbStep(climber);
  private final Command c_regressClimbStep    = new RegressClimbStep(climber);
  private final Command c_stopClimber         = new StopClimber(climber);
  // private final Command c_controlClimbMotors  = new ControlLeanboiMotors(climber, () -> operator.getLeftY(), () -> operator.getRightY());

  // ? Turret
  private final Command c_aimTurret           = new AimAndControlTurret2(turret, () -> operator.getRightX());
  private final Command c_toggleTurretControl = new ToggleTurretControlState(turret);
  // private final Command c_aimTurret = new AimAndControlTurret2(turret, () -> superman.getRightX());

  // * macros
  private final Command c_runIntake = new IntakeAndIndex(intake, indexer);
  private final Command c_runOutake = new ExtendAndOuttake(intake, indexer);

  private final Command c_runShooterLow   = new AutoShootAndIndex(shooter, indexer, turret, 760.0);
  // private final Command c_runShooterClose = new ShootAndIndex(shooter, indexer, 1200.0);
  // private final Command c_runShooterMid   = new ShootAndIndex(shooter, indexer, 1600.0);
  private final Command c_runShooterTarmak = new AutoShootAndIndex(shooter, indexer, turret, 1740.0);
  private final Command c_runShooterPZ    = new AutoShootAndIndex(shooter, indexer, turret, 1930.0); // 2100.0
  private final Command c_autoShoot       = new AutoShootAndIndex(shooter, indexer, turret);
  // private final Command c_runShooterFar   = new ShootAndIndex(shooter, indexer, 2500.0);

  private final Command c_runClimbStep = new RunClimbStep(turret, climber, shooter);
  private final Command c_runClimbStep2 = new RunClimbStep2(turret, climber, shooter);

  // * Utility
  private final Command c_stopShooterDefault = new InstantCommand(() -> shooter.setDefaultCommand(
    new StopShooter(shooter)
  ), shooter);

  private final Command c_idleShooterDefault = new InstantCommand(() -> shooter.setDefaultCommand(
    new IdleShooter(shooter)
  ), shooter);

  /* Trajectories */
  private PathPlannerTrajectory tr_test_1, tr_test_2, tr_test_3,
    tr_two_ball_r1, tr_two_ball_r2, tr_two_ball_r3,
    tr_two_ball_b1, tr_two_ball_b2, tr_two_ball_b3,
    tr_four_ball_r1_1, tr_four_ball_r1_2, tr_four_ball_r1_3,
    tr_five_ball_r1_1, tr_five_ball_r1_2, tr_five_ball_r1_3, tr_five_ball_r1_4, tr_five_ball_r1_5,
    tr_five_ball_a_r1_1, tr_five_ball_a_r1_2, tr_five_ball_a_r1_3, tr_five_ball_a_r1_4;

  private enum Autons { Nothing, TwoBall, FiveBall_R1, FiveBall_A_R1 }
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
        // superman,
        driver,
        true, true
      )
    );

    shooter.setDefaultCommand(c_idleShooter);

    intake.setDefaultCommand(c_retract);

    indexer.setDefaultCommand(c_stopIndexer);

    turret.setDefaultCommand(c_aimTurret);

    climber.setDefaultCommand(c_stopClimber);
  }

  private void configureButtonBindings () {

    // * driver
    DR_circle.whenPressed(c_zeroGyro);
    
    // * operator
    OP_leftBumper.whileHeld(c_runIntake);
    OP_rightBumper.whileHeld(c_runShooterTarmak);
    OP_circle.whenPressed(c_progressClimbStep);
    OP_triangle.whileHeld(c_toggleTurretControl);
    OP_square.whileHeld(c_runOutake);
    OP_cross.whenPressed(c_runClimbStep2);
    OP_big.whileHeld(c_autoShoot);
    OP_options.whileHeld(c_runShooterPZ);

    OP_up.whileHeld(c_runShooterLow);
    OP_down.whenPressed(c_regressClimbStep);
    OP_left.whenPressed(c_idleShooterDefault);
    OP_right.whenPressed(c_stopShooterDefault);

    // * superman
    // S_square.whileHeld(c_runIntake);
    // S_circle.whileHeld(c_runOutake);
    // S_circle.whileHeld(c_runShooterPZ);
    // S_triangle.whenPressed(c_zeroGyro);
    // S_cross.whileHeld(c_autoShoot);

    // S_left.whenPressed(c_progressClimbStep);
    // S_right.whenPressed(c_runClimbStep);
    // S_down.whenPressed(c_regressClimbStep);

    // S_down.whenPressed(new SetPosition(climber, 0.0));
    // S_up.whenPressed(new SetPosition(climber, 180_000.0));
    // S_left.whenPressed(new SetPosition(climber, 100_000.0));
    // S_right.whenPressed(new SetPosition(climber, 80_000.0));
  }

  private void loadTrajectories () {
    tr_test_1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("test_1", 1.0, 1.0);
    tr_test_2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("test_2", 1.0, 1.0);
    tr_test_3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("test_3", 1.0, 1.0);

    tr_two_ball_r1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball_r1", 1.0, 1.0);
    tr_two_ball_r2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball_r2", 1.0, 1.0);
    tr_two_ball_r3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball_r3", 1.0, 1.0);

    tr_two_ball_b1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball_b1", 1.0, 1.0);
    tr_two_ball_b2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball_b2", 1.0, 1.0);
    tr_two_ball_b3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("two_ball_b3", 1.0, 1.0);

    tr_four_ball_r1_1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("four_ball_r1_1", 1.0, 1.0);
    tr_four_ball_r1_2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("four_ball_r1_2", 1.0, 1.0);
    tr_four_ball_r1_3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("four_ball_r1_3", 1.0, 1.0);

    tr_five_ball_r1_1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("five_ball_r1_1", 1.0, 1.0);
    tr_five_ball_r1_2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("five_ball_r1_2", 3.0, 2.7); // 3.0 2.7
    tr_five_ball_r1_3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("five_ball_r1_3", 5.0, 2.7); // 3.0 2.7
    tr_five_ball_r1_4 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("five_ball_r1_4", 6.0, 6.0); // 5.0 3.0
    tr_five_ball_r1_5 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("five_ball_r1_5", 8.0, 8.0);

    tr_five_ball_a_r1_1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("five_ball_a_r1_1", 7.0, 7.0);
    tr_five_ball_a_r1_2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("five_ball_a_r1_2", 10.0, 10.0);
    tr_five_ball_a_r1_3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("five_ball_a_r1_3", 10.0, 8.0);
    tr_five_ball_a_r1_4 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("five_ball_a_r1_4", 12.0, 12.0);
  }

  private void configureAuton () {
    autonChooser.setDefaultOption("Nothing", Autons.Nothing);
    autonChooser.addOption("2Ball", Autons.TwoBall);
    autonChooser.addOption("5Ball R1", Autons.FiveBall_R1);
    autonChooser.addOption("5Ball A R1", Autons.FiveBall_A_R1);

    ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    tab.add("Auton Chooser", autonChooser);
  }

  private void dashboardStuff () {
    ShuffleboardTab tab = Shuffleboard.getTab("Climber");
    tab.addNumber("PSI", () -> hub.getPressure(0));
  }

  // public Command getAutonomousCommand () {
  //   return new RunPathPlannerTrajectory(drivetrain, tr_test_3);
  //   return new FiveBallR1(drivetrain, intake, indexer, shooter, tr_five_ball_r1_1, tr_five_ball_r1_2, tr_five_ball_r1_3, tr_five_ball_r1_4);
  // }

  public Command getAutonomousCommand () {
    switch (autonChooser.getSelected()) {
      case Nothing:
        return new WaitCommand(15.0);

      case TwoBall:
        return new TwoBallR1(drivetrain, intake, indexer, shooter, tr_two_ball_r1);

      case FiveBall_R1:
        return new FiveBallR1(drivetrain, intake, indexer, shooter, tr_five_ball_r1_1, tr_five_ball_r1_2, tr_five_ball_r1_3, tr_five_ball_r1_4, tr_five_ball_r1_5);
    
      case FiveBall_A_R1:
        return new FiveBallAR1(drivetrain, intake, indexer, shooter, tr_five_ball_a_r1_1, tr_five_ball_a_r1_2, tr_five_ball_a_r1_3, tr_five_ball_a_r1_4);

      default:
        return new WaitCommand(15.0);
    }
  }
}
