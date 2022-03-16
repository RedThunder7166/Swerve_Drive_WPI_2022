// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class InnerArmCommand extends CommandBase {

  private final ClimberSubsystem m_climberSubsystem;
  private final DoubleSupplier m_innerClimb;
  private final DoubleSupplier m_innerBS;
  /** Creates a new InnerArmCommand. */
  public InnerArmCommand(ClimberSubsystem subsystem, DoubleSupplier innerClimb, DoubleSupplier innerBS) {
    m_climberSubsystem = subsystem;
    m_innerClimb = innerClimb;
    m_innerBS = innerBS;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_climberSubsystem.driveArms(m_innerClimb.getAsDouble(), 0, m_innerBS.getAsDouble(), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.driveArms(0, 0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
