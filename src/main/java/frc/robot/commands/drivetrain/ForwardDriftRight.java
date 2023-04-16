// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.SwerveDrivetrain;
// import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** An example command that uses an example subsystem. */
public class ForwardDriftRight extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrivetrain swerve;
  public SwerveModuleState forward;
  public SwerveModuleState backward;
  public SwerveModuleState advancing;

  public SwerveModuleState stop;

  public Rotation2d rotation;
//   public SwerveModuleState[] states = new SwerveModuleState[4];

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ForwardDriftRight(SwerveDrivetrain swerve) {
    this.swerve = swerve;
    rotation = new Rotation2d(0);
    forward = new SwerveModuleState(0.7,rotation);
    backward = new SwerveModuleState(-0.2,rotation);
    advancing = new SwerveModuleState(2,rotation);

    stop = new SwerveModuleState(0,rotation);

    SwerveModuleState[] states = {forward, forward, forward, forward};
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState[] forwardArray = {forward, forward, forward, forward};
    SwerveModuleState[] backwardArray = {backward, backward, backward, backward};
    SwerveModuleState[] advancingArray = {advancing, advancing, advancing, advancing};
    SwerveModuleState[] stopState = {stop, stop, stop, stop};

    int count = 0;
    // swerve.setModuleStates(states);

    // while(Math.abs(swerve.getGyroRoll()) <2){
    //     swerve.setModuleStates(advancingArray);
    // }

    // while(Math.abs(swerve.getGyroRoll()) >6){
    //     // new ForwardSlow2(swerve);
    //     if(swerve.getGyroRoll() > 8){
    //         swerve.setModuleStates(forwardArray);
    //     }
    //     // else if(swerve.getGyroRoll() < -4){
    //     //     swerve.setModuleStates(backwardArray);
    //     //     count++;
    //     // }
    //     else{
    //         swerve.setModuleStates(stopState);
    //     }
    // }

    // while(swerve.getGyroRoll() < -4 ){
    //   swerve.setModuleStates(backwardArray);
    // }

    // while((swerve.getGyroRoll()) > 7 && swerve.getGyroRoll() < 16){
    //   RunPathPlannerTrajectory2()
    // }

    // for(int i = 0; i<160; i++){
    //   swerve.setModuleStates(backwardArray);
    // }

    // int x = 0;
    // while(x <160){
    //   swerve.setModuleStates(backwardArray);
    //   x++;
    // }





    // swerve.setModuleStates(stopState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
