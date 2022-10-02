// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerIntakeSubsystem;

public class AutoIntake extends CommandBase {
  private final IndexerIntakeSubsystem m_indexerIntakeSubsystem;
  private final double m_speed;
  private final Timer m_timer = new Timer();
  /** Creates a new AutoIntake. */
  public AutoIntake(IndexerIntakeSubsystem index, double speed) {
    m_indexerIntakeSubsystem = index;
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_indexerIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexerIntakeSubsystem.driveIndexerIntake(0, m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerIntakeSubsystem.driveIndexerIntake(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_indexerIntakeSubsystem.getBrokenBeam() == true  && m_timer.get() < 4){
      return false;
    } else{
      return true;
    }
  }
}
