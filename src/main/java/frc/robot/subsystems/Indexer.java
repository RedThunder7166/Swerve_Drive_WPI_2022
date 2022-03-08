// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private final WPI_TalonFX indexerMotor = new WPI_TalonFX(20);
  private final WPI_TalonFX anotherMotor = new WPI_TalonFX(21); //FIXME change the CAN ID

  public void indexerMotor(double speed){
    anotherMotor.setVoltage(speed * 12);
    indexerMotor.setVoltage(speed * 12);
  }

  /** Creates a new Indexer. */
  public Indexer() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
