// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;



public class ShootAdjustAuto extends CommandBase {

  private final VisionSubsystem m_visionSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final LEDSubsystem m_ledSubsystem;
  private final Timer m_timer = new Timer();
  /** Creates a new ShootingAdjustCommand. */
  public ShootAdjustAuto(VisionSubsystem subsystem, DriveSubsystem subsystem2, LEDSubsystem led) {
    m_visionSubsystem = subsystem;
    m_driveSubsystem = subsystem2;
    m_ledSubsystem = led;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
    addRequirements(m_visionSubsystem);
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
    double range = 1; // Range in which it will adjust
    double yaw = m_visionSubsystem.getHubYaw();
    double pidOutput = m_visionSubsystem.getAimPIDCaculation();

    if(m_visionSubsystem.hubCameraHasTargets() == true){
      m_ledSubsystem.setLED(.77);
      if(yaw > range){
        m_driveSubsystem.drive(0, 0, pidOutput, true);
      } else if(yaw < -1 *range){
        m_driveSubsystem.drive(0, 0, pidOutput, true);
      } else {
        m_driveSubsystem.drive(0, 0, 0, true);
      }
    } else{
        m_driveSubsystem.drive(0, 0, 0, true);
        m_ledSubsystem.setLED(-.05);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0,0,0,true);
    m_ledSubsystem.setLED(-.25);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double yaw = m_visionSubsystem.getHubYaw();

    if(Math.abs(yaw) < 1 || m_timer.get() > 3){
      return true;
    } else{
      return false;
    }
  }
}
