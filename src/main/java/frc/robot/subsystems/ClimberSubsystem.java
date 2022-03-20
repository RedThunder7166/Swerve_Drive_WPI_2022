// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */

  private final WPI_TalonFX innerClimbMotor = new WPI_TalonFX(Constants.MechanismConstants.kInnerClimbMotor);
  private final WPI_TalonFX outerClimbMotor = new WPI_TalonFX(Constants.MechanismConstants.kOuterClimbMotor);
  private final WPI_TalonFX bsOuterLeftMotor = new WPI_TalonFX(Constants.MechanismConstants.kBSOuterLeftMotor);
  private final WPI_TalonFX bsOuterRightMotor = new WPI_TalonFX(Constants.MechanismConstants.kBSOuterRightMotor);
  private final WPI_TalonFX bsInnerLeftMotor = new WPI_TalonFX(Constants.MechanismConstants.kBSInnerLeftMotor);
  private final WPI_TalonFX bsInnerRightMotor = new WPI_TalonFX(Constants.MechanismConstants.kBSInnerRightMotor); 

  public void driveInnerClimb(double speed){
    innerClimbMotor.setVoltage(speed * 12);
  }
//TODO: comment postive and negative directions
  public void driveArms(double innerClimb, double outerClimb, double innerBS, double outerBS){




    innerClimbMotor.setVoltage(Math.pow(innerClimb, 2) * 12);
    outerClimbMotor.setVoltage(Math.pow(outerClimb, 2) * 12);

    //BSinner
    bsInnerLeftMotor.setVoltage(Math.pow(innerBS, 2) * 12);
    bsInnerRightMotor.setVoltage(Math.pow(innerBS, 2) * -12); //MUST BE OPPOSITE SIGN - BS motors are at opposite ends

    //BSouter
    bsOuterLeftMotor.setVoltage(Math.pow(outerBS, 2) * 12);
    bsOuterRightMotor.setVoltage(Math.pow(outerBS, 2) * -12); //MUST BE OPPOSITE SIGN - BS motors are at opposite ends

  }

  public ClimberSubsystem() {

    innerClimbMotor.setNeutralMode(NeutralMode.Brake);
    outerClimbMotor.setNeutralMode(NeutralMode.Brake);
    bsOuterLeftMotor.setNeutralMode(NeutralMode.Brake);
    bsOuterRightMotor.setNeutralMode(NeutralMode.Brake);
    bsInnerLeftMotor.setNeutralMode(NeutralMode.Brake);
    bsInnerRightMotor.setNeutralMode(NeutralMode.Brake);

    innerClimbMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 500);
    outerClimbMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 500);
    bsOuterLeftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 500);
    bsOuterRightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 500);
    bsInnerLeftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 500);
    bsInnerRightMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 500);

    innerClimbMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 500);
    outerClimbMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 500);
    bsOuterLeftMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 500);
    bsOuterRightMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 500);
    bsInnerLeftMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 500);
    bsInnerRightMotor.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 500);

    // innerClimbMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 50);
    // outerClimbMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 50);
    // bsOuterLeftMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 50);
    // bsOuterRightMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 50);
    // bsInnerLeftMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 50);
    // bsInnerRightMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 50);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
