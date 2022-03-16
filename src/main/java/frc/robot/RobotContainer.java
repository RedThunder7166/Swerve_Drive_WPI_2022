// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IndexerIntakeCommand;
import frc.robot.commands.InnerArmCommand;
import frc.robot.commands.OuterArmCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final IndexerIntakeSubsystem m_indexerIntakeSubsystem = new IndexerIntakeSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  XboxController m_climberController = new XboxController(OIConstants.kClimberControllerPort);

  Button DR_A_Button = new JoystickButton(m_driverController, 1);
  Button DR_B_Button = new JoystickButton(m_driverController, 2);
  Button DR_X_Button = new JoystickButton(m_driverController, 3);
  Button DR_Y_Button = new JoystickButton(m_driverController, 4);
  Button DR_LB_Button = new JoystickButton(m_driverController, 5);
  Button DR_RB_Button = new JoystickButton(m_driverController, 6);
  Button DR_Select_Button = new JoystickButton(m_driverController, 7);
  Button DR_Start_Button = new JoystickButton(m_driverController, 8);
  Button DR_Left_Stick_Button = new JoystickButton(m_driverController, 9);
  Button DR_Right_Stick_Button = new JoystickButton(m_driverController, 10);
  
  Button OP_A_Button = new JoystickButton(m_operatorController, 1);
  Button OP_B_Button = new JoystickButton(m_operatorController, 2);
  Button OP_X_Button = new JoystickButton(m_operatorController, 3);
  Button OP_Y_Button = new JoystickButton(m_operatorController, 4);
  Button OP_LB_Button = new JoystickButton(m_operatorController, 5);
  Button OP_RB_Button = new JoystickButton(m_operatorController, 6);
  Button OP_Select_Button = new JoystickButton(m_operatorController, 7);
  Button OP_Start_Button = new JoystickButton(m_operatorController, 8);
  Button OP_Left_Stick_Button = new JoystickButton(m_operatorController, 9);
  Button OP_Right_Stick_Button = new JoystickButton(m_operatorController, 10);

  // The first argument is the root container
  // The second argument is whether logging and config should be given separate tabs


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
      // A split-stick arcade command, with forward/backward controlled by the left
      // hand, and turning controlled by the right
      new RunCommand(
        () -> 
          m_robotDrive.drive(
            modifyAxis(m_driverController.getLeftY()) // xAxis
            * DriveConstants.kMaxSpeedMetersPerSecond, 
            modifyAxis(m_driverController.getLeftX()) // yAxis
            * DriveConstants.kMaxSpeedMetersPerSecond, 
            modifyAxis(m_driverController.getRightX()) // rot
            * DriveConstants.kMaxRotationalSpeedMetersPerSecond, 
            true),
            
        m_robotDrive));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
        // Back button zeros the gyroscope
        new Button(m_driverController::getBackButton)
        // No requirements because we don't need to interrupt anything
        .whenPressed(m_robotDrive::zeroHeading);

        OP_LB_Button.whenHeld(new InnerArmCommand(m_climberSubsystem, 
                                                  () -> m_operatorController.getRightY(), 
                                                  () -> m_operatorController.getLeftY()));
        OP_RB_Button.whenHeld(new OuterArmCommand(m_climberSubsystem, 
                                                  () -> m_operatorController.getRightY(), 
                                                  () -> m_operatorController.getLeftY()));
        OP_Select_Button.toggleWhenPressed(new IndexerIntakeCommand(m_indexerIntakeSubsystem, 
                                                                    ()-> m_operatorController.getLeftY(),
                                                                    () -> m_operatorController.getRightY()));

  }









  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = 
      new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory exampleTrajectory = 
      TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(new Translation2d(1, 0)),
        new Pose2d(1, 1, new Rotation2d(90)),
        config);

    /** thetaController is for the x, y, and theta motions of the robot
     * Error for x/y is meters, theta is degrees/radians
     * Tune them by making solo x/y trajectories as well as theta trajectories
    */

    var thetaController = 
      new ProfiledPIDController(
        AutoConstants.kPThetaController, 0 , 0, AutoConstants.kThetaControllerConstraints);
    
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
      new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, //Functional interface to feed the supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at end
      
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Cube the axis
    value = Math.copySign(value * value * value, value);

    return value;
  }
}


