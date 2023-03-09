// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;



import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class photonvisionSubsystem extends SubsystemBase {
  /** Creates a new photonvisionSubsystem. */

  PhotonCamera camera;
  private boolean hasTarget;
  PhotonTrackedTarget target;
  



  public photonvisionSubsystem() {
    camera = new PhotonCamera(getName());

    var results = camera.getLatestResult();
    PhotonTrackedTarget target = results.getBestTarget();
    hasTarget = results.hasTargets();
   

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

//   public void printTest(){
//     System.out.println(target.getYaw());
// }

}
