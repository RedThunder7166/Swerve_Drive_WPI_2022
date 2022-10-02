// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerIntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootHighAuto extends CommandBase {

  private final VisionSubsystem m_visionSubsystem;
  private final IndexerIntakeSubsystem m_indexerIntakeSubsystem;
  private final Timer m_timer = new Timer();

  /** Creates a new ShootHighCommand. */
  public ShootHighAuto(VisionSubsystem visionSubsystem, IndexerIntakeSubsystem indexerIntakeSubsystem) {
    m_visionSubsystem = visionSubsystem;
    m_indexerIntakeSubsystem = indexerIntakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
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
    m_indexerIntakeSubsystem.shootHigh(.75); //Changed from 0.8 to 0.75 Kettering Kickoff
    m_indexerIntakeSubsystem.driveIntake(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexerIntakeSubsystem.shootHigh(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_timer.get() < 3){
      return false;
    } else{
      return true;
    }
  }
}
