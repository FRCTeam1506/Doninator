// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.TelescopingSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.drivetrain.SwerveTeleop;
// import frc.robot.commands.shooter.IdleShooter;
// import frc.robot.commands.shooter.Shoot;
// import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OurBeautifulGlowingCANdleSubsystem;
import frc.robot.utils.TrajectoryHelper;

public class RobotContainer {


  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0);
  public static final PS4Controller operator = new PS4Controller(1);


  /* Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kSquare.value);
  // private final JoystickButton shoot = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
  private final JoystickButton pneumaticButtonUp = new JoystickButton(operator, PS4Controller.Button.kTriangle.value);
  private final JoystickButton pneumaticButtonMid = new JoystickButton(operator, PS4Controller.Button.kCircle.value);
  private final JoystickButton pneumaticButtonDown = new JoystickButton(operator, PS4Controller.Button.kSquare.value);

  private final JoystickButton outtakeButton = new JoystickButton(operator, PS4Controller.Button.kR2.value);
  private final JoystickButton intakeButton = new JoystickButton(operator, PS4Controller.Button.kL2.value);

  private final JoystickButton telescopeForward = new JoystickButton(operator, PS4Controller.Button.kL1.value);
  private final JoystickButton telescopeBack = new JoystickButton(operator, PS4Controller.Button.kR1.value);
  private final JoystickButton telescopePrint = new JoystickButton(operator, PS4Controller.Button.kTouchpad.value);
  private final JoystickButton telescopeMid = new JoystickButton(operator, PS4Controller.Button.kShare.value);
  private final JoystickButton telescopeHigh = new JoystickButton(operator, PS4Controller.Button.kOptions.value);


  private final JoystickButton candlePurple = new JoystickButton(operator, PS4Controller.Button.kL3.value);
  private final JoystickButton candleYellow = new JoystickButton(operator, PS4Controller.Button.kR3.value);


  public double yAxis = -operator.getRawAxis(1);

  /* Deadbands */
  // yAxis = (Math.abs(yAxis) < DEADBAND) ? 0 : yAxis;




  /* Subsystems */
  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  // private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem(hub);
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final frc.robot.subsystems.TelescopingSubsystem telescope = new frc.robot.subsystems.TelescopingSubsystem();
  private final OurBeautifulGlowingCANdleSubsystem candle = new OurBeautifulGlowingCANdleSubsystem();

  public static PneumaticHub hub      = new PneumaticHub();
  public static Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  /* Commands */
  private final Command c_zeroGyro = new InstantCommand( () -> drivetrain.zeroGyro() );
  // private final Command c_shoot = new Shoot(shooter, 1850.0);
  private final Command c_runPneumaticUp = new InstantCommand( () -> arm.setHigh());
  private final Command c_runPneumaticMid = new InstantCommand( () -> arm.setMid());
  private final Command c_runPneumaticDown = new InstantCommand( () -> arm.setLow());
  double intakePower = 0.25;
  public final Command c_runIntake = new InstantCommand( () -> intake.intake(intakePower));
  private final Command c_outtakeIntake = new InstantCommand( () -> intake.intake(-intakePower));
  private final Command c_stopIntake = new InstantCommand( () -> intake.stop());

  private final Command c_TelescopeForward = new InstantCommand( () -> telescope.forward());
  private final Command c_TelescopeBack = new InstantCommand( () -> telescope.backward());
  private final Command c_TelescopeStop = new InstantCommand( () -> telescope.stop());
  private final Command c_TelescopeMid = new InstantCommand( () -> telescope.setMid());
  private final Command c_TelescopeHigh = new InstantCommand( () -> telescope.setHigh());
  private final Command c_TelescopePrintStuff = new InstantCommand( () -> telescope.printStuff());

  private final Command c_candleYellow = new InstantCommand( () -> candle.yellow());
  private final Command c_candlePurple = new InstantCommand( () -> candle.purple());


  /* Trajectories */
  public PathPlannerTrajectory tr_test_2;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LiveWindow.disableAllTelemetry();
    DriverStation.silenceJoystickConnectionWarning(true);

    hub.enableCompressorAnalog(100, 120);

    yAxis = -operator.getRawAxis(1);

    setDefaultCommands();
    configureButtonBindings();
    loadTrajectories();
  }

  private void configureButtonBindings() {
    zeroGyro.onTrue(c_zeroGyro);
    // new JoystickButton(driver, PS4Controller.Button.kTriangle.value).whileHeld(c_shoot);
    // shoot.whileHeld(c_shoot);
    pneumaticButtonUp.onTrue(c_runPneumaticUp);
    pneumaticButtonMid.onTrue(c_runPneumaticMid);
    pneumaticButtonDown.onTrue(c_runPneumaticDown);

    outtakeButton.onTrue(c_outtakeIntake);
    intakeButton.onTrue(c_runIntake);

    outtakeButton.onFalse(c_stopIntake);
    intakeButton.onFalse(c_stopIntake);

    telescopeForward.onTrue(c_TelescopeForward);
    telescopeBack.onTrue(c_TelescopeBack);

    telescopeForward.onFalse(c_TelescopeStop);
    telescopeBack.onFalse(c_TelescopeStop);
    telescopePrint.onTrue(c_TelescopePrintStuff);
    telescopeMid.onTrue(c_TelescopeMid);
    telescopeHigh.onTrue(c_TelescopeHigh);

    candleYellow.onTrue(c_candleYellow);
    candlePurple.onTrue(c_candlePurple);
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(
      new SwerveTeleop(
        drivetrain,
        driver,
        true, true
      )
    );

    // intake.setDefaultCommand(new IntakeCommand(intake, operator));

    // shooter.setDefaultCommand(new IdleShooter(shooter));
  }

  private void loadTrajectories() {
    // tr_test = TrajectoryHelper.loadWPILibTrajectoryFromFile("test1");
    // tr_straight = TrajectoryHelper.loadWPILibTrajectoryFromFile("straight");
    // tr_holotest = TrajectoryHelper.loadPathPlannerTrajectory("straight2");
    tr_test_2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("test_2");
  }



  public Command getAutonomousCommand(PathPlannerTrajectory name) {
    return new RunPathPlannerTrajectory2(drivetrain, name);
  }

  // @Override
  // public void execute(){
  //   configureButtonBindings();
  // }
}
