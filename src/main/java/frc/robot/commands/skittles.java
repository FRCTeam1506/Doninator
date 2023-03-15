// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.OurBeautifulGlowingCANdleSubsystem;
import frc.robot.subsystems.TelescopingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class skittles extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final OurBeautifulGlowingCANdleSubsystem candle;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public skittles(OurBeautifulGlowingCANdleSubsystem candle) {
    this.candle = candle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(candle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    candle.gsa();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
