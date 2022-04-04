// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */ 

  private final PWMTalonFX leftClimbMotor = new PWMTalonFX(Constants.MechanismConstants.kInnerClimbMotor);
  private final PWMTalonFX rightClimbMotor = new PWMTalonFX(Constants.MechanismConstants.kOuterClimbMotor);
  private final DigitalInput rightArmInput = new DigitalInput(0);


  //prevents the inner climb arms from breaking the orange line again.  ROBOT WILL NOT MOVE INNER
  //ARMS IF THE LIMIT SWITCH IS TRIPPED

//TODO: comment postive and negative directions
  public void driveArms(double speed){
    if(rightArmInput.get() == false && speed < 0){
      leftClimbMotor.setVoltage(0);
      rightClimbMotor.setVoltage(0);
    } else {
      leftClimbMotor.setVoltage(speed * 12);
      rightClimbMotor.setVoltage(speed * 12);
    }

  }

  public ClimberSubsystem() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("DIO 0", rightArmInput.get());
    // This method will be called once per scheduler run
  }
}
