// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerIntakeSubsystem;
import frc.robot.subsystems.ThetaController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallAuto extends SequentialCommandGroup {
  /** Creates a new OneBallAuto. */
  public OneBallAuto(DriveSubsystem m_driveSubsystem, IndexerIntakeSubsystem m_indexerIntakeSubsystem, ThetaController m_thetaController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Shoot the ball, wait 3 seconds
      new SimpleShootCommand(m_indexerIntakeSubsystem), 
      //Reset odometry for initial pose
      new InstantCommand(()-> m_driveSubsystem.resetOdometry(Constants.AutoConstants.movingOutTrajectory.getInitialPose())), 
      //Drive backwards 1.5 meters
      new SwerveControllerCommand(
        Constants.AutoConstants.movingOutTrajectory,
        m_driveSubsystem::getPose, //Functional interface to feed the supplier
        Constants.DriveConstants.kDriveKinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        m_thetaController.getThetaController(),
        m_driveSubsystem::setModuleStates,
        m_driveSubsystem)
    );
  }
}
