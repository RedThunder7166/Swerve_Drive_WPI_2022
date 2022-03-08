// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */


 DigitalInput interiorLimitSwitch1 = new DigitalInput(0);
 DigitalInput interiorLimitSwitch2; 

 DigitalInput exteriorLimitSwitch3;
 DigitalInput exteriorLimitSwitch4;

 private final WPI_TalonFX interiorArm = new WPI_TalonFX(23);//FIXME Change CAN id all below
 private final WPI_TalonFX interiorTrackMotor = new WPI_TalonFX(24);
 private final WPI_TalonFX exteriorArm = new WPI_TalonFX(25);
 private final WPI_TalonFX exteriorTrackMotor = new WPI_TalonFX(26);

private final PIDController m_trackPID = new PIDController(.1, 0, 0);
 
 public void interiorArm(double speed){
   interiorArm.setVoltage(speed * 12);
 }
 public void interiorTrack(double speed){
  interiorTrackMotor.setVoltage(speed * 12); 
}
 public void exteriorArm(double speed){
   exteriorArm.setVoltage(speed * 12);
 }
 public void exteriorTrack(double speed){
   exteriorTrackMotor.setVoltage(speed * 12);
 }

 public void setInteriorTrackPosition(double metersPosition){

  interiorTrackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
  //Converts enocder position of motor to inches
  // FIXME: Change to SI units for PID controller
  double currentPositionMeters = interiorTrackMotor.getSelectedSensorPosition() * (.1/2048) * .0254;

  double pidInteriorOutput = m_trackPID.calculate(currentPositionMeters, metersPosition);

  // ASSUMES clicked = true
  // Assumes positive is away from limit swtich 1
  // Assumes postiive is towards limit switch 2
  
  if(interiorLimitSwitch1.get() == true && pidInteriorOutput > 0){
    interiorTrackMotor.setVoltage(pidInteriorOutput);
  } else if (interiorLimitSwitch2.get() == true && pidInteriorOutput < 0){
    interiorTrackMotor.setVoltage(pidInteriorOutput);
  } else if (interiorLimitSwitch1.get() == false && interiorLimitSwitch2.get() == false){
    interiorTrackMotor.setVoltage(pidInteriorOutput);
  } else{
    interiorTrackMotor.setVoltage(0);}
  }
  

  public void setExteriorTrackPosition(double metersPosition){

    exteriorTrackMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    //Converts enocder position of motor to inches
    // FIXME: Change to SI units for PID controller
    double currentPositionMeters = exteriorTrackMotor.getSelectedSensorPosition() * (.1/2048) * .0254;
  
    double pidExteriorOutput = m_trackPID.calculate(currentPositionMeters, metersPosition);
    
    // ASSUMES clicked = true
    // Assumes positive is away from limit swtich 1
    // Assumes postiive is towards limit switch 2
    
    if(exteriorLimitSwitch3.get() == true && pidExteriorOutput > 0){
      exteriorTrackMotor.setVoltage(pidExteriorOutput);
    } else if (exteriorLimitSwitch4.get() == true && pidExteriorOutput < 0){
      exteriorTrackMotor.setVoltage(pidExteriorOutput);
    } else if (exteriorLimitSwitch3.get() == false && exteriorLimitSwitch4.get() == false){
      exteriorTrackMotor.setVoltage(pidExteriorOutput);
    } else{
      exteriorTrackMotor.setVoltage(0);
    }
    
   

 }




  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
