// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
// import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class BackwardsSlow extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrivetrain swerve;
  public SwerveModuleState backward;
  public Rotation2d rotation;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BackwardsSlow(SwerveDrivetrain swerve) {
    this.swerve = swerve;
    rotation = new Rotation2d(0);
    backward = new SwerveModuleState(-0.2,rotation);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState[] backwardArray = {backward, backward, backward, backward};
    swerve.setModuleStates(backwardArray);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
