// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerIntakeSubsystem;

public class IndexerIntakeCommand extends CommandBase {

  private final IndexerIntakeSubsystem m_indexerIntakeSubsystem;
  private final DoubleSupplier m_indexer;
  private final DoubleSupplier m_intake;
  /** Creates a new IndexerIntakeCommand. */
  public IndexerIntakeCommand(IndexerIntakeSubsystem subsystem, DoubleSupplier indexer, DoubleSupplier intake) {
    m_indexerIntakeSubsystem = subsystem;
    m_indexer = indexer;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexerIntakeSubsystem.driveIndexerIntake(m_indexer.getAsDouble(), m_intake.getAsDouble());
    m_indexerIntakeSubsystem.setIntakeActive(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerIntakeSubsystem.setIntakeActive(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
