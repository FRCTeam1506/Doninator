// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
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
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.subsystems.TelescopingSubsystem;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.drivetrain.SwerveTeleop;
import frc.robot.commands.macros.ground;
import frc.robot.commands.macros.high;
import frc.robot.commands.macros.mid;
import frc.robot.commands.telescoping.SetLow;
// import frc.robot.commands.shooter.IdleShooter;
// import frc.robot.commands.shooter.Shoot;
// import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.AllignedSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OurBeautifulGlowingCANdleSubsystem;
import frc.robot.utils.TrajectoryHelper;
import frc.robot.commands.auton.RA1;
import frc.robot.commands.auton.RA100;
import frc.robot.commands.auton.RA200;

public class RobotContainer {


  /* Controllers */
  private final PS4Controller driver = new PS4Controller(0);
  public static final PS4Controller operator = new PS4Controller(1);

  public static boolean manualControls;

  /* Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kCircle.value);

  // private final JoystickButton shoot = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
  private final JoystickButton pneumaticButtonUp = new JoystickButton(operator, PS4Controller.Button.kTriangle.value);
  private final JoystickButton pneumaticButtonMid = new JoystickButton(operator, PS4Controller.Button.kCircle.value);
  private final JoystickButton pneumaticButtonDown = new JoystickButton(operator, PS4Controller.Button.kSquare.value);

  private final JoystickButton outtakeButton = new JoystickButton(operator, PS4Controller.Button.kL2.value);
  private final JoystickButton intakeButton = new JoystickButton(operator, PS4Controller.Button.kR2.value);

  private final JoystickButton telescopeForward = new JoystickButton(operator, PS4Controller.Button.kL1.value);
  private final JoystickButton telescopeBack = new JoystickButton(operator, PS4Controller.Button.kR1.value);
  private final JoystickButton telescopePrint = new JoystickButton(operator, PS4Controller.Button.kTouchpad.value);
  private final JoystickButton telescopeMid = new JoystickButton(operator, PS4Controller.Button.kShare.value);
  private final JoystickButton telescopeHigh = new JoystickButton(operator, PS4Controller.Button.kOptions.value);


  private final JoystickButton candlePurple = new JoystickButton(operator, PS4Controller.Button.kL3.value);
  private final JoystickButton candleYellow = new JoystickButton(operator, PS4Controller.Button.kR3.value);
  private final JoystickButton candleIncrement = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
  private final JoystickButton candleRainbow = new JoystickButton(driver, PS4Controller.Button.kCross.value);

  private final JoystickButton telescopeRun = new JoystickButton(operator, PS4Controller.Button.kPS.value);
  private final JoystickButton telescopeZero = new JoystickButton(operator, PS4Controller.Button.kCross.value);
  private final JoystickButton telescopePrint2 = new JoystickButton(driver, PS4Controller.Button.kShare.value);


  private final JoystickButton macroLow = new JoystickButton(operator, PS4Controller.Button.kSquare.value);
  private final JoystickButton macroMid = new JoystickButton(operator, PS4Controller.Button.kCircle.value);
  private final JoystickButton macroHigh = new JoystickButton(operator, PS4Controller.Button.kTriangle.value);
  private final JoystickButton macroHP = new JoystickButton(operator, PS4Controller.Button.kPS.value);
  private final JoystickButton macroTransport = new JoystickButton(operator, PS4Controller.Button.kTouchpad.value);

  private final JoystickButton operatorOptions = new JoystickButton(operator, PS4Controller.Button.kOptions.value);


  private final JoystickButton switchMode = new JoystickButton(driver, PS4Controller.Button.kTouchpad.value);



  // private final JoystickButton telescopeZeroWHILE = new JoystickButton(driver, PS4Controller.Button.kPS.value);

  int operatorPOV = operator.getPOV();


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
  private final AllignedSubsystem macro = new AllignedSubsystem(arm, telescope);

  public static PneumaticHub hub      = new PneumaticHub();
  public static Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  /* Commands */
  private final Command c_zeroGyro = new InstantCommand( () -> drivetrain.zeroGyro() );
  // private final Command c_shoot = new Shoot(shooter, 1850.0);
  private final Command c_runPneumaticUp = new InstantCommand( () -> arm.setHigh());
  private final Command c_runPneumaticMid = new InstantCommand( () -> arm.setMid());
  private final Command c_runPneumaticDown = new InstantCommand( () -> arm.setLow());
  double intakePower = 0.5;
  public final Command c_runIntake = new InstantCommand( () -> intake.intakeDefSpeed());
  private final Command c_outtakeIntake = new InstantCommand( () -> intake.outtakeDefSpeed());
  private final Command c_stopIntake = new InstantCommand( () -> intake.stop());

  private final Command c_TelescopeForward = new InstantCommand( () -> telescope.forward());
  private final Command c_TelescopeBack = new InstantCommand( () -> telescope.backward());
  private final Command c_TelescopeStop = new InstantCommand( () -> telescope.stop());
  private final Command c_TelescopeReset = new InstantCommand( () -> telescope.resetMotors());
  private final Command c_TelescopePrintStuff = new InstantCommand( () -> telescope.printStuff());

  private final Command c_TelescopeLow = new InstantCommand( () -> telescope.runZero());
  private final Command c_TelescopeMid = new InstantCommand( () -> telescope.runMid());
  private final Command c_TelescopeHigh = new InstantCommand( () -> telescope.runHigh());
  private final Command c_TelescopeHP = new InstantCommand( () -> telescope.runHP());

  private final Command c_candleYellow = new InstantCommand( () -> candle.yellow());
  private final Command c_candlePurple = new InstantCommand( () -> candle.purple());
  private final Command c_candleIncrement = new InstantCommand( () -> candle.incrementColor(-5));
  private final Command c_candleRainbow = new InstantCommand( () -> candle.gsa());

  private final Command c_macroLow = new InstantCommand( () -> macro.ground());
  private final Command c_macroMid = new InstantCommand( () -> macro.mid());
  private final Command c_macroHigh = new InstantCommand( () -> macro.high());
  private final Command c_macroTransport = new InstantCommand( () -> macro.transport());
  private final Command c_macroHP = new InstantCommand( () -> macro.HP());
  private final Command c_macroStop = new InstantCommand( () -> macro.stop());

  private final Command c_switchMode = new InstantCommand( () -> switchMode());




  // private final Command c_timedTelescope = new InstantCommand( () -> telescope.testRun());
  // private final Command c_telescopeZeroWHILE = new InstantCommand (() -> telescope.runZeroWhile());


  /* Trajectories */
  public PathPlannerTrajectory go_straight, one_one, B_LW1, B_LW2, B_RW1, B_RW2, B_CT1, B_CT2, B_CT3, R_LW1, R_LW2, R_RW1, R_RW2, R_CT1, R_CT2, R_CT3, B_RW100, B_LW100, B_CT100, B_CT101, R_RW100, R_LW100, R_CT100, R_CT101, B_STR, R_STR, RL_STR, RL_STR2;
  private enum Colors { None, Red, Blue }
  private enum Autons { Nothing, LeftWing, Center, RightWing, RW100, LW100, CT100, CT101, STR }
  private SendableChooser<Colors> colorChooser = new SendableChooser<>();
  private SendableChooser<Autons> autonChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LiveWindow.disableAllTelemetry();
    DriverStation.silenceJoystickConnectionWarning(true);

    hub.enableCompressorAnalog(100, 120);

    yAxis = -operator.getRawAxis(1);

    setDefaultCommands();
    configureButtonBindings();
    loadTrajectories();
    configureAuton();
    dashboardStuff();
    checkOperatorPOV();

    // if(manualControls == true){
    //   configureManualButtonBindings();
    // }
    // else{
    //   configureAutoButtonBindings();
    // }

    UsbCamera cam = CameraServer.startAutomaticCapture();
    cam.setResolution(1280, 720);
    cam.setFPS(20);

  }

  private void switchMode(){
    System.out.println("switch mode");
    
    manualControls = !manualControls;
  }

  public void configureButtonBindings(){
    zeroGyro.onTrue(c_zeroGyro);

    outtakeButton.onTrue(c_outtakeIntake);
    intakeButton.onTrue(c_runIntake);
    outtakeButton.onFalse(c_stopIntake);
    intakeButton.onFalse(c_stopIntake);
    telescopePrint2.onTrue(c_TelescopePrintStuff);

    // telescopeForward.onFalse(c_TelescopeStop);
    // telescopeBack.onFalse(c_TelescopeStop);

    candleYellow.onTrue(c_candleYellow);
    candlePurple.onTrue(c_candlePurple);
    // candleIncrement.onTrue(c_TelescopeReset);
    candleRainbow.onTrue(c_candleRainbow);

    switchMode.onTrue(c_switchMode);

  }
  private void configureManualButtonBindings() {
    // new JoystickButton(driver, PS4Controller.Button.kTriangle.value).whileHeld(c_shoot);
    pneumaticButtonUp.onTrue(c_runPneumaticUp);
    pneumaticButtonMid.onTrue(c_runPneumaticMid);
    pneumaticButtonDown.onTrue(c_runPneumaticDown);

    telescopeForward.onTrue(c_TelescopeForward);
    telescopeBack.onTrue(c_TelescopeBack);

    telescopeForward.onFalse(c_TelescopeStop);
    telescopeBack.onFalse(c_TelescopeStop);
    telescopePrint.onTrue(c_TelescopePrintStuff);

    telescopeRun.onTrue(c_TelescopeLow);
    telescopeMid.onTrue(c_TelescopeMid);
    telescopeHigh.onTrue(c_TelescopeHigh);

    // telescopeZero.onTrue(c_TelescopeHP);


    // telescopeRun.onTrue(c_timedTelescope);
    // telescopeZeroWHILE.onTrue(c_telescopeZeroWHILE);

    // checkOperatorPOV();
  }

  private void configureAutoButtonBindings() {
    macroLow.onTrue(c_macroLow);
    macroMid.onTrue(c_macroMid);
    macroHigh.onTrue(c_macroHigh);
    macroHP.onTrue(c_macroHP);
    macroTransport.onTrue(c_macroTransport);
    macroTransport.onFalse(c_macroStop);
    macroLow.onFalse(c_macroStop);
    macroMid.onFalse(c_macroStop);
    macroHigh.onFalse(c_macroStop);
    macroHP.onFalse(c_macroStop);

  }


  public void checkOperatorPOV(){
    if(operator.getPOV() == 270){
      System.out.println("hello270");
      macro.ground();
    }
    else if(operator.getPOV() == 90){
      System.out.println("hello90");
      macro.mid();
    }
    else if(operator.getPOV() == 0){
      macro.high();
      System.out.println("hello0");
    }
    else if(operator.getPOV() == 180){
      macro.transport();
    }

    operatorOptions.onTrue(c_macroHP);
    operatorOptions.onFalse(c_macroStop);



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

  }

  private void loadTrajectories() {
    // tr_test = TrajectoryHelper.loadWPILibTrajectoryFromFile("test1");
    // go_straight = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("testpark",3.0,3.0);
    // one_one = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("testpark",3.0,3.0);

    go_straight = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("testpark",3.0,3.0, false);
    
    // LW0 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("LW0",3.0,3.0, false);
    // LW1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("LW1",3.0,3.0, false);
    // LW2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("LW2",3.0,3.0, false);
    // LW3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("LW3",3.0,3.0, false);

    // RedLeft1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("RedLeft1",3.0,3.0, true);
    // RedLeft2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("RedLeft2",3.0,3.0, true);
    // RedLeft3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("RedLeft3",3.0,3.0, true);

    B_LW1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_LW1",3.0,3.0, false);
    B_LW2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_LW2",3.0,3.0, false);
    B_RW1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_RW1",3.0,3.0, false);
    B_RW2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_RW2",3.0,3.0, false);
    
    B_CT1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_CT1",3.0,3.0, false);
    B_CT2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_CT2",3.0,3.0, false);
    B_CT3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_CT3",3.0,3.0, false);


    R_LW1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_LW1",3.0,3.0, true);
    R_LW2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_LW2",3.0,3.0, true);
    R_RW1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_RW1",3.0,3.0, true);
    R_RW2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_RW2",3.0,3.0, true);

    R_CT1 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_CT1",3.0,3.0, true);
    R_CT2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_CT2",3.0,3.0, true);
    R_CT3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("B_CT3",3.0,3.0, true);

    B_LW100 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("LW100",3.0,3.0, false);
    B_RW100 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("RW100",3.0,3.0, false);
    B_CT100 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("CT100",1.0,1.0, false);
    B_CT101 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("CT101",1.0,1.0, false);

    R_LW100 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("LW100",3.0,3.0, true);
    R_RW100 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("RW100",3.0,3.0, true);
    R_CT100 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("CT100",1.0,1.0, true);
    R_CT101 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("CT101",1.0,1.0, true);

    B_STR = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("STR",2.0,0.5, false);
    RL_STR = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("RL_STR",4.0,2, false);
    RL_STR2 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory("RL_STR2",4.0,2, false);


  }

  private void configureAuton() {
    autonChooser.setDefaultOption("Nothing", Autons.Nothing);
    autonChooser.addOption("LeftWing", Autons.LeftWing);
    autonChooser.addOption("Center", Autons.Center);
    autonChooser.addOption("RightWing", Autons.RightWing);
    autonChooser.addOption("LW100", Autons.LW100);
    autonChooser.addOption("RW100", Autons.RW100);
    autonChooser.addOption("CT100", Autons.CT101);
    autonChooser.addOption("CT101", Autons.CT101);
    autonChooser.addOption("STR", Autons.STR);


    colorChooser.setDefaultOption("Nothing", Colors.None);
    colorChooser.addOption("RED", Colors.Red);
    colorChooser.addOption("BLUE", Colors.Blue);



    ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    tab.add("Color Chooser", colorChooser);
    tab.add("Auton Chooser", autonChooser);
  }

  private void dashboardStuff () {
    ShuffleboardTab tab = Shuffleboard.getTab("LiftingArm");
    tab.addNumber("PSI", () -> hub.getPressure(0));
  }

  public Command getAutonomousCommand() {

    switch (colorChooser.getSelected()){

      case None:
        return new WaitCommand(15.0);


      case Red:
        switch (autonChooser.getSelected()) {
          case Nothing:
            return new WaitCommand(15.0);

          case LeftWing:
            return new RA1(drivetrain, intake, telescope, arm, R_LW1, R_LW2, null);
          
          case RightWing:
            return new RA1(drivetrain, intake, telescope, arm, R_RW1, R_RW2, null);
        
          case Center:
            return new RA1(drivetrain, intake, telescope, arm, R_CT1, R_CT2, R_CT3);

          case LW100:
            return new RA100(drivetrain, intake, telescope, arm, R_LW100);
            
          case RW100:
            return new RA100(drivetrain, intake, telescope, arm, R_RW100);
            
          case CT100:
            return new RA100(drivetrain, intake, telescope, arm, R_CT100);

          case CT101:
            return new RA100(drivetrain, intake, telescope, arm, R_CT101);

          case STR:
            // return new RA1(drivetrain, intake, telescope, arm, R_STR, B_STR, null);
            // return new RunPathPlannerTrajectory2(drivetrain, B_STR);
              return new RA200(drivetrain, intake, telescope, arm, RL_STR, RL_STR2);

        
          default:
            return new WaitCommand(15.0);
        }


      case Blue:
        switch (autonChooser.getSelected()) {
          case Nothing:
            return new WaitCommand(15.0);

          case LeftWing:
            return new RA1(drivetrain, intake, telescope, arm, B_LW1, B_LW2, null);
          
          case RightWing:
            return new RA1(drivetrain, intake, telescope, arm, B_RW1, B_RW2, null);
        
          case Center:
            return new RA1(drivetrain, intake, telescope, arm, B_CT1, B_CT2, B_CT3);

          case LW100:
            return new RA100(drivetrain, intake, telescope, arm, B_LW100);
            
          case RW100:
            return new RA100(drivetrain, intake, telescope, arm, B_RW100);
            
          case CT100:
            return new RA100(drivetrain, intake, telescope, arm, B_CT100);

          case CT101:
            return new RA100(drivetrain, intake, telescope, arm, B_CT101);

          case STR:
            return new RA1(drivetrain, intake, telescope, arm, B_STR, R_STR, null);



          default:
            return new WaitCommand(15.0);
        }


        default:
          return new WaitCommand(15.0);
    }
    
    //return new RunPathPlannerTrajectory2(drivetrain, name);
  }

  public void periodic(){
    switchMode.onTrue(c_switchMode);
    if(manualControls == true){
      configureManualButtonBindings();
    }
    else{
      configureAutoButtonBindings();
    }

  }

}
//na