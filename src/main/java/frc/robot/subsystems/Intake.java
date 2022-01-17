// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase { //Start at the class name, enter what physical objects you have

  private WPI_TalonFX intakeMotor = new WPI_TalonFX(Constants.MechanismConstants.kIntakeMotor); 
   
  private double kPIntake = .5;




  public void setIntakeMotor(double speed){
    intakeMotor.setVoltage(speed * 12 * kPIntake);
  }

  /** Creates a new Intake. */

  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
