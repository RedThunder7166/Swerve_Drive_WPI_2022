// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */ 

  private final PWMTalonFX leftClimbMotor = new PWMTalonFX(Constants.MechanismConstants.kInnerClimbMotor);
  private final PWMTalonFX rightClimbMotor = new PWMTalonFX(Constants.MechanismConstants.kOuterClimbMotor);


  //prevents the inner climb arms from breaking the orange line again.  ROBOT WILL NOT MOVE INNER
  //ARMS IF THE LIMIT SWITCH IS TRIPPED

//TODO: comment postive and negative directions
  public void driveArms(double speed){
    leftClimbMotor.setVoltage(speed * 12);
    rightClimbMotor.setVoltage(speed * 12);

  }

  public ClimberSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
