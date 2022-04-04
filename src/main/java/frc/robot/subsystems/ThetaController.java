// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;

public class ThetaController extends SubsystemBase {
  /** Creates a new ThetaController. */

  public static ProfiledPIDController thetaController;

  public ThetaController() {
    thetaController = new ProfiledPIDController(
                      AutoConstants.kPThetaController, 0 , 0, AutoConstants.kThetaControllerConstraints);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public ProfiledPIDController getThetaController(){
    return thetaController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
