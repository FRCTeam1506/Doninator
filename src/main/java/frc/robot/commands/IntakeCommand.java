// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


/** An example command that uses an example subsystem. */
public class IntakeCommand extends CommandBase {
  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public final IntakeSubsystem m_intake;
  private PS4Controller controller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand(IntakeSubsystem intake, PS4Controller controller) {
    m_intake = intake;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {

  // }

  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   System.out.println("hello");
  //   double yAxiss = -controller.getRawAxis(1);

  //   RobotContainer robot = new RobotContainer();
  //   m_intake.intake(yAxiss);
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.intake(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
