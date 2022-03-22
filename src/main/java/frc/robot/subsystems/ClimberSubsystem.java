// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */ 

  private final PWMTalonFX innerClimbMotor = new PWMTalonFX(Constants.MechanismConstants.kInnerClimbMotor);
  private final PWMTalonFX outerClimbMotor = new PWMTalonFX(Constants.MechanismConstants.kOuterClimbMotor);
  private final PWMTalonFX bsOuterLeftMotor = new PWMTalonFX(Constants.MechanismConstants.kBSOuterLeftMotor);
  private final PWMTalonFX bsOuterRightMotor = new PWMTalonFX(Constants.MechanismConstants.kBSOuterRightMotor);
  private final PWMTalonFX bsInnerLeftMotor = new PWMTalonFX(Constants.MechanismConstants.kBSInnerLeftMotor);
  private final PWMTalonFX bsInnerRightMotor = new PWMTalonFX(Constants.MechanismConstants.kBSInnerRightMotor);

  public void driveInnerClimb(double speed){
    innerClimbMotor.setVoltage(speed * 12);
  }
//TODO: comment postive and negative directions
  public void driveArms(double innerClimb, double outerClimb, double innerBS, double outerBS){

    


    innerClimbMotor.setVoltage(Math.pow(innerClimb, 3) * 12);
    outerClimbMotor.setVoltage(Math.pow(outerClimb, 3) * 12);

    //BSinner
    bsInnerLeftMotor.setVoltage(Math.pow(innerBS, 3) * 12);
    bsInnerRightMotor.setVoltage(Math.pow(innerBS, 3) * -12); //MUST BE OPPOSITE SIGN - BS motors are at opposite ends

    //BSouter
    bsOuterLeftMotor.setVoltage(Math.pow(outerBS, 3) * 12);
    bsOuterRightMotor.setVoltage(Math.pow(outerBS, 3) * -12); //MUST BE OPPOSITE SIGN - BS motors are at opposite ends

  }

  public ClimberSubsystem() {

  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
