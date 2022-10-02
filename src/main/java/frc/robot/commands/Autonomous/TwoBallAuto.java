// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ThetaController;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  public TwoBallAuto(DriveSubsystem m_driveSubsystem, IndexerIntakeSubsystem m_indexerIntakeSubsystem, VisionSubsystem m_visionSubsystem, ThetaController m_thetaController, LEDSubsystem m_ledSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> m_driveSubsystem.resetOdometry(Constants.AutoConstants.grabSecondBallTrajectory.getInitialPose())), 
      new ParallelCommandGroup(
        new SwerveControllerCommand(
          Constants.AutoConstants.grabSecondBallTrajectory,
          m_driveSubsystem::getPose, //Functional interface to feed the supplier
          Constants.DriveConstants.kDriveKinematics,
          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          m_thetaController.getThetaController(),
          m_driveSubsystem::setModuleStates,
          m_driveSubsystem),
        new AutoIntake(m_indexerIntakeSubsystem, 1)
      ),
    new InstantCommand(()-> m_driveSubsystem.resetOdometry(Constants.AutoConstants.rotate90Trajectory.getInitialPose())), 
    new SwerveControllerCommand(
      Constants.AutoConstants.rotate90Trajectory,
      m_driveSubsystem::getPose,
      Constants.DriveConstants.kDriveKinematics,
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      m_thetaController.getThetaController(),
      m_driveSubsystem::setModuleStates,
      m_driveSubsystem),
    //new ShootAdjustAuto(m_visionSubsystem, m_driveSubsystem, m_ledSubsystem),
    new ShootHighAuto(m_visionSubsystem, m_indexerIntakeSubsystem)
      
    );
  }
}
