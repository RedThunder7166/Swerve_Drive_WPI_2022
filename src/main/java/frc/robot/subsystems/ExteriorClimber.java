// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExteriorClimber extends SubsystemBase {


private final WPI_TalonFX climbMotorExterior = new WPI_TalonFX(Constants.MechanismConstants.kExteriorClimb);
private final WPI_TalonFX trackMotorExterior = new WPI_TalonFX(Constants.MechanismConstants.kExteriorTrack);

private final DigitalInput rearLimitSwitch = new DigitalInput(0);
private final DigitalInput frontLimitSwitch = new DigitalInput(1);



private double kPClimberExterior = .5;


public void setClimbMotorExterior(double speed){
  if (speed < 0) {
    if (frontLimitSwitch.get() == true) {
       climbMotorExterior.setVoltage(0);
    } else {climbMotorExterior.setVoltage(speed);}
  } else if (speed > 0) {
    if(rearLimitSwitch.get() == true) {
      climbMotorExterior.setVoltage(0);
    } else{climbMotorExterior.setVoltage(speed);}
  } else {
    climbMotorExterior.setVoltage(0);
  }
}


public void setTrackMotorExterior(double speed){
  if (speed < 0) {
    if (frontLimitSwitch.get() == true) {
       trackMotorExterior.setVoltage(0);
  } else {trackMotorExterior.setVoltage(speed);}
}else if (speed > 0) {
    if(rearLimitSwitch.get() == true) {
      trackMotorExterior.setVoltage(0);
    } else {climbMotorExterior.setVoltage(speed);}
  } else {
      climbMotorExterior.setVoltage(0);
  }

      trackMotorExterior.setVoltage(speed * 12 * kPClimberExterior);
}

      
  


public double getTrackPositionExteriorInches(){

  return trackMotorExterior.getSelectedSensorPosition() / 20480;
}

public void setTrackPositionExerior(double positionInches){

  double m_headingError = getTrackPositionExteriorInches() - positionInches;

}

public double getClimbPositionExteriorInches(){

  return climbMotorExterior.getSelectedSensorPosition();
}

public void setClimbPostitionExterior(double positionInches){

  double m_headingError = getClimbPositionExteriorInches() - positionInches;

}


  /** Creates a new ExteriorClimber. */
  public ExteriorClimber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
