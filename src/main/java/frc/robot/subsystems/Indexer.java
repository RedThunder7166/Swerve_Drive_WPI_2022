// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private final WPI_TalonFX rearIndexerMotorFalcon = new WPI_TalonFX(22); 
  private final CANSparkMax frontIndexerMotor = new CANSparkMax(21, MotorType.kBrushless);

  //Spark 5676 RPM
  //Falcon 6380 RPM

  private final double kNeoFalconRatio = 5676/6380;
  private final double kp = .6;

  //Set speed for rear indexer motor
  // TODO: Comment positive/negative direction
  public void driveRearIndexer(double speed){
    rearIndexerMotorFalcon.setVoltage(speed * 12 * kNeoFalconRatio);
  }

  //Set speed for front indexer motor
  // TODO: Comment postive/negative direction
  public void driveFrontIndexer(double speed){
    frontIndexerMotor.setVoltage(speed * 12); 
  }

  public void driveIndexer(double speed){
    rearIndexerMotorFalcon.setVoltage(speed * 12 * kp);
    frontIndexerMotor.setVoltage(speed * 12 * kp);
  }


  /** Creates a new Indexer. */
  public Indexer() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
