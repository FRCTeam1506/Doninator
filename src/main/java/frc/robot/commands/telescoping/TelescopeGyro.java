// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescoping;

import frc.robot.Constants.TelescopingSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TelescopeGyro extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final frc.robot.subsystems.TelescopingSubsystem telescope;
  private final SwerveDrivetrain drivetrain;
  private boolean finished = false;
  private boolean mid = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TelescopeGyro(frc.robot.subsystems.TelescopingSubsystem telescope, SwerveDrivetrain drivetrain) {
    this.telescope = telescope;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(!mid){
        telescope.runMid();
        mid = true;
      }
      if(Math.abs(drivetrain.getGyroAngleDegrees()) > 120 && Math.abs(drivetrain.getGyroAngleDegrees()) < 300){
        telescope.runHighAuto();
      }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
