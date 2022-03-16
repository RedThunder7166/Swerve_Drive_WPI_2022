// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class OuterArmCommand extends CommandBase {

  private final ClimberSubsystem m_climberSubsystem;
  private final DoubleSupplier m_outerClimb;
  private final DoubleSupplier m_outerBS;
  /** Creates a new OuterArmCommand. */
  public OuterArmCommand(ClimberSubsystem subsystem, DoubleSupplier outerClimb, DoubleSupplier outerBS) {

    m_climberSubsystem = subsystem;
    m_outerClimb = outerClimb;
    m_outerBS = outerBS;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubsystem.driveArms(0, m_outerClimb.getAsDouble(), 0, m_outerBS.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
