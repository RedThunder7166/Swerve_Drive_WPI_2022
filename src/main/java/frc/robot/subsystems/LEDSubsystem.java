// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

   //LED Blinkin controllers
   private Spark m_leftBlinkin = new Spark(8);
   private Spark m_rightBlinkin = new Spark(9);

   public void setLeftLED(double value){
     m_leftBlinkin.set(value);
   }

   public void setRightLED(double value){
     m_rightBlinkin.set(value);
   }

   public void setLED(double value){
     m_leftBlinkin.set(value);
     m_rightBlinkin.set(value);
   }
   
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
