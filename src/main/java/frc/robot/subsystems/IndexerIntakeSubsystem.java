// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerIntakeSubsystem extends SubsystemBase {

  

  private final PWMTalonFX rearIndexerMotorFalcon = new PWMTalonFX(Constants.MechanismConstants.krearIndexerMotorFalcon);
  private final PWMTalonFX intakeMotor = new PWMTalonFX(Constants.MechanismConstants.kIntakeMotor);
  private final PWMSparkMax frontIndexerMotor = new PWMSparkMax(Constants.MechanismConstants.kfrontIndexerMotor);
  
  


  //Spark 5676 RPM
  //Falcon 6380 RPM

  private final double kNeoFalconRatio = .89;
  private final double kp_indexer = .3;
  private final double kp_intake = .6;
  private final double kDeadband = .1;
  private double indexerSpeed = 0;
  private final double kp_indexer_auto = .72;

  /** Creates a new IndexerIntakeSubsystem. */
  public IndexerIntakeSubsystem() {}


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

  //.65

  public void shootHighAuto(double indexerAuto){
    rearIndexerMotorFalcon.setVoltage(indexerAuto * 12 * kp_indexer_auto * kNeoFalconRatio);
    frontIndexerMotor.setVoltage(indexerAuto * 12 * kp_indexer_auto);
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
