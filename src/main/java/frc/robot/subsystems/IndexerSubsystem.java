// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

private final WPI_TalonFX Indexer1 = new WPI_TalonFX(Constants.MechanismConstants.kIndexerMotor1);
private final WPI_TalonFX Indexer2 = new WPI_TalonFX(Constants.MechanismConstants.kIndexerMotor2);

private double kPIndexer = .5;





public void setIndexer1(double speed){
  Indexer1.setVoltage(speed * 12 *kPIndexer);
}
public void setIndexer2(double speed){
  Indexer2.setVoltage(speed * 12 *kPIndexer);
}

public void setIndexerMotors(double speed){
  Indexer1.setVoltage(speed * 12 *kPIndexer);
  Indexer2.setVoltage(speed * 12 *kPIndexer);
}



  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
