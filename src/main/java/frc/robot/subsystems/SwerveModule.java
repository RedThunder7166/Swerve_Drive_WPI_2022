// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonFX m_turningMotor;

  private final CANCoder m_turnEncoder;
  // Driving encoder uses the integrated FX encoder
  // e.g. testMotor.getSelectedSensorPosition();

  // PID controller for velocity. DO NOT SET kD.  It is redundant as setVoltage() already controls this
  private final PIDController m_drivePIDController = 
    new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);
  


  //Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = 
    new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController, 
      0,
      ModuleConstants.kDModuleTurningController,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
  

  SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
    DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter);

  SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
    DriveConstants.ksTurning, DriveConstants.kvTurning);


  /** Creates a new SwerveModule. **/
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderPorts,
      double angleZero) {
    
    // Initialize the motors
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel);

    // Configure current lmits for motors - prevents disabling/brownout
    m_driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5));
    //m_driveMotor.configClosedloopRamp(0.25);
    m_driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 45, 0.5));
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    
    // Configure the encoders for both motors
    
    m_driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    this.m_turnEncoder = new CANCoder(turningEncoderPorts);
    this.m_turnEncoder.configMagnetOffset(-1 * angleZero);
    this.m_turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    this.m_turnEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100);

    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
    m_turningMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);

    m_turningMotor.setNeutralMode(NeutralMode.Brake);
    
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

  }

  public double getModuleHeading(){
    double m_turning = this.m_turnEncoder.getAbsolutePosition();

    return m_turning;
  }

  //Returns the current state of the module

  public SwerveModuleState getState(){
    double m_speedMetersPerSecond = 
      ModuleConstants.kDrivetoMetersPerSecond * m_driveMotor.getSelectedSensorVelocity();

    double m_turningRadians =  
      ((2*Math.PI)/360) * m_turnEncoder.getAbsolutePosition();

    return new SwerveModuleState(m_speedMetersPerSecond, new Rotation2d(m_turningRadians));
  }

  public void setDesiredState(SwerveModuleState desiredState){

    double m_speedMetersPerSecond = 
      ModuleConstants.kDrivetoMetersPerSecond * m_driveMotor.getSelectedSensorVelocity();

    double m_turningRadians =  
      ((2*Math.PI)/360) * m_turnEncoder.getAbsolutePosition();
    
    //Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = 
      SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningRadians));

    //Calculate the drive output from the drive PID controller
    final double driveOutput =
      m_drivePIDController.calculate(m_speedMetersPerSecond, state.speedMetersPerSecond)
      + driveFeedforward.calculate(state.speedMetersPerSecond);


    final var turnOutput = 
      m_turningPIDController.calculate(m_turningRadians, state.angle.getRadians())
      + turnFeedForward.calculate(m_turningPIDController.getSetpoint().velocity);

    // Calculate the turning motor output from the turning PID controller
    m_driveMotor.setVoltage(driveOutput); 
    m_turningMotor.setVoltage(turnOutput);  
  }


  public void resetEncoders() {
    m_turnEncoder.setPosition(0);
    m_driveMotor.setSelectedSensorPosition(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}