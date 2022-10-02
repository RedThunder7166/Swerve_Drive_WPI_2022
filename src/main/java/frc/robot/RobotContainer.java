// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ShootHighCommand;
import frc.robot.commands.ShootingAdjustCommand;
import frc.robot.commands.Autonomous.OneBallAuto;
import frc.robot.commands.Autonomous.TwoBallAuto;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerIntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ThetaController;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Boolean used to display if the conveyor system is active
  
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final IndexerIntakeSubsystem m_indexerIntakeSubsystem = new IndexerIntakeSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final ThetaController m_thetaController = new ThetaController();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  SendableChooser<Trajectory> m_chooser = new SendableChooser<>();
  SendableChooser<Command> m_commandChooser = new SendableChooser<>();
  
  

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

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
            * DriveConstants.kMaxSpeedMetersPerSecond * -1, 
            modifyAxis(m_driverController.getLeftX()) // yAxis
            * DriveConstants.kMaxSpeedMetersPerSecond * -1, 
            modifyAxis(m_driverController.getRightX() * -1) // rot
            * DriveConstants.kMaxRotationalSpeedMetersPerSecond + m_driverController.getRawAxis(3)*-1.3 - m_driverController.getRawAxis(2)*-1.3, 
            true),
            
        m_robotDrive));

    Shuffleboard.getTab("Autonomous").add(m_chooser);    

    Shuffleboard.getTab("Autonomous").add(m_commandChooser);
    m_commandChooser.setDefaultOption("1 Ball Auto", new OneBallAuto(m_robotDrive, m_indexerIntakeSubsystem, m_thetaController));
    m_commandChooser.addOption("2 Ball Auto", new TwoBallAuto(m_robotDrive, m_indexerIntakeSubsystem, m_visionSubsystem, m_thetaController, m_ledSubsystem));

    m_climberSubsystem.setDefaultCommand(
      new RunCommand(
        () -> 
          m_climberSubsystem.driveArms(
            m_operatorController.getRawAxis(3) - m_operatorController.getRawAxis(2))
          , m_climberSubsystem)
    );

    m_indexerIntakeSubsystem.setDefaultCommand(
      new RunCommand(
        () -> 
          m_indexerIntakeSubsystem.driveIndexerIntake(
            m_operatorController.getLeftY(),
            m_operatorController.getRightY()), 
          m_indexerIntakeSubsystem)
    );

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

        DR_Y_Button.whenHeld(new ShootingAdjustCommand(m_visionSubsystem, m_robotDrive));
        OP_RB_Button.whenHeld(new ShootHighCommand(m_visionSubsystem, m_indexerIntakeSubsystem));

  }




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

      
    return m_commandChooser.getSelected();
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


