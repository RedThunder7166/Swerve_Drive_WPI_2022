// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerIntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootHighCommand extends CommandBase {

  private final VisionSubsystem m_visionSubsystem;
  private final IndexerIntakeSubsystem m_indexerIntakeSubsystem;
  /** Creates a new ShootHighCommand. */
  public ShootHighCommand(VisionSubsystem visionSubsystem, IndexerIntakeSubsystem indexerIntakeSubsystem) {
    m_visionSubsystem = visionSubsystem;
    m_indexerIntakeSubsystem = indexerIntakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexerIntakeSubsystem.shootHigh(m_visionSubsystem.calculateLaunchSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerIntakeSubsystem.shootHigh(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
