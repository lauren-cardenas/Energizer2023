// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class armSubsystem extends SubsystemBase {
  WPI_VictorSPX m_arm;

  private DigitalInput armSwitchDown = new DigitalInput(MechConstants.mArmDownSwitch);
  private DigitalInput armSwitchUp = new DigitalInput(MechConstants.m_armSwitchUp);
 
  public armSubsystem() {
   m_arm = new WPI_VictorSPX(MechConstants.m_armPort);
  }
 
  @Override
  public void periodic() {
  }
 
  public void armRun(double speed) {
   m_arm.set(speed);
  }
 /*
  public void armReverse(double speed){
   m_arm.set(speed);
  }
 */

  public boolean getStatusUp() {
    return armSwitchUp.get();
  }

  public boolean getStatusDown(){
    return armSwitchDown.get();
  }

}