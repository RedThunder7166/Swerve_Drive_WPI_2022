// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera("photonvision");
  private final PIDController m_aimPIDController = new PIDController(.08, .05, 0); // FIXME: Adjust KP for aiming
  
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {

  }

  //Negative yaw means target is to the left
  //Positive yaw means target is to the right
  public double getHubYaw(){
    var result = camera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();
    
    if(result.hasTargets()){
      return targets.get(0).getYaw();
    } else {
      return 0;
    }
  }

  public double getAimPIDCaculation(){
    return m_aimPIDController.calculate(getHubYaw(), 0);
  }

  public double getHubArea(){
    var result = camera.getLatestResult();
    List<PhotonTrackedTarget> targets = result.getTargets();

    if(result.hasTargets()){
      return targets.get(0).getArea();
    } else {
      return 0;
    }
  }

  public boolean hubCameraHasTargets(){
    var result = camera.getLatestResult();
    return result.hasTargets();
  }

  public double calculateLaunchSpeed(){
      return .8;
    
  }

  public boolean isRedAlliance(){
    if(DriverStation.getAlliance() == Alliance.Red){
      return true;
    } else{
      return false;
    }
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Target?", hubCameraHasTargets());

    // This method will be called once per scheduler run
  }
}
