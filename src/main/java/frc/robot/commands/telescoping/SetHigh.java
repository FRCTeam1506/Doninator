// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescoping;

import frc.robot.subsystems.TelescopingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SetHigh extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TelescopingSubsystem Telescoping;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetHigh(TelescopingSubsystem Telescoping) {
    this.Telescoping = Telescoping;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Telescoping);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Telescoping.runHP();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
