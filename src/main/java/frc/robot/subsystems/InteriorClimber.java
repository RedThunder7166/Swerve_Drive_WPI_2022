// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class InteriorClimber extends SubsystemBase {

  private final WPI_TalonFX climbMotorInterior = new WPI_TalonFX(Constants.MechanismConstants.kInteriorClimb);
  private final WPI_TalonFX trackMotorInterior = new WPI_TalonFX(Constants.MechanismConstants.kInteriorTrack);

  private final DigitalInput rearLimitSwitch = new DigitalInput(0);
  private final DigitalInput frontLimitSwitch = new DigitalInput(1);


  private double kPClimberInterior = .5;


  public void setClimbMotorInterior(double speed){
    if (speed < 0) {
      if (frontLimitSwitch.get() == true) {
         climbMotorInterior.setVoltage(0);}
    } else if (speed > 0) {
      if(rearLimitSwitch.get() == true) {
        climbMotorInterior.setVoltage(0);}
    } else {
    climbMotorInterior.setVoltage(speed * 12 * kPClimberInterior);
    }
  }

  public void setTrackMotorInterior(double speed){
    if (speed < 0) {
      if (frontLimitSwitch.get() == true) {
         trackMotorInterior.setVoltage(0);}
    } else if (speed > 0) {
      if(rearLimitSwitch.get() == true) {
        trackMotorInterior.setVoltage(0);}
    } else {
    trackMotorInterior.setVoltage(speed * 12 * kPClimberInterior);
    }
  }
  



  /** Creates a new InteriorClimber. */
  public InteriorClimber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
