// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerIntakeSubsystem extends SubsystemBase {

  private final WPI_TalonFX rearIndexerMotorFalcon = new WPI_TalonFX(Constants.MechanismConstants.krearIndexerMotorFalcon); 
  private final CANSparkMax frontIndexerMotor = new CANSparkMax(Constants.MechanismConstants.kfrontIndexerMotor, MotorType.kBrushless);
  private final WPI_TalonFX intakeMotor = new WPI_TalonFX(20); 
  


  //Spark 5676 RPM
  //Falcon 6380 RPM

  private final double kNeoFalconRatio = .89;
  private final double kp_indexer = .6;
  private final double kp_intake = .6;
  private final double kDeadband = .1;
  private double indexerSpeed = 0;

  /** Creates a new IndexerIntakeSubsystem. */
  public IndexerIntakeSubsystem() {
    // Change status frames of unimportant motors to reduce CAN usage
    rearIndexerMotorFalcon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);
    rearIndexerMotorFalcon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 1000);

    frontIndexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 1000);
    frontIndexerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 1000);
    
  }

  public void driveIndexerIntake(double indexer, double intake){


    

    if (Math.abs(indexer) < kDeadband){
      indexerSpeed = 0;
    } else {
      indexerSpeed = indexer;
    }

    rearIndexerMotorFalcon.setVoltage(indexerSpeed * 12 * kp_indexer * kNeoFalconRatio);
    frontIndexerMotor.setVoltage(indexerSpeed * 12 * kp_indexer);

    intakeMotor.setVoltage(intake * 12 * kp_intake);
  }

  // Publishes on the dashboard 
  public void setIntakeActive(boolean activity){
    SmartDashboard.putBoolean("Intake Active", activity);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
