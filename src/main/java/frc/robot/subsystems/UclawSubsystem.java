// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class UclawSubsystem extends SubsystemBase {
  /** Creates a new CclawSubsystem. */
  WPI_VictorSPX m_Uclaw;
  public UclawSubsystem() {
    m_Uclaw = new WPI_VictorSPX (MechConstants.m_UclawPort);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void UclawRun(double speed){
    m_Uclaw.set(-speed);
  }
  
}
