// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;



public class ShootingAdjustCommand extends CommandBase {

  private final VisionSubsystem m_visionSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  /** Creates a new ShootingAdjustCommand. */
  public ShootingAdjustCommand(VisionSubsystem subsystem, DriveSubsystem subsystem2) {
    m_visionSubsystem = subsystem;
    m_driveSubsystem = subsystem2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
    addRequirements(m_visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double range = 1.0; // Range in which it will adjust
    double yaw = m_visionSubsystem.getYaw();
    double pidOutput = m_visionSubsystem.getAimPIDCaculation();
    SmartDashboard.putBoolean("Has Target?", m_visionSubsystem.cameraHasTargets());

    if(m_visionSubsystem.cameraHasTargets() == true){
      
      if(yaw > range){
        m_driveSubsystem.drive(0, 0, pidOutput, true);
      } else if(yaw < -1 *range){
        m_driveSubsystem.drive(0, 0, pidOutput, true);
      }
    } else{}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0,0,0,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
