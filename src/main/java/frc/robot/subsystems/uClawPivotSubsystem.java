// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechConstants;

public class uClawPivotSubsystem extends SubsystemBase {
  /** Creates a new uClawPivotSubsystem. */
  WPI_VictorSPX m_uClawPivot;

  private DigitalInput m_pivotDown = new DigitalInput(MechConstants.m_pivotDownSwitch);
  private DigitalInput m_pivotUp = new DigitalInput(MechConstants.m_pivotUpSwitch);
  private DigitalInput m_pivotShoot = new DigitalInput(MechConstants.m_pivotShootSwitch);
  
  public uClawPivotSubsystem() {
    m_uClawPivot = new WPI_VictorSPX(MechConstants.m_UclawPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void UclawPivotRun(double speed){
    m_uClawPivot.set(speed);
  }

  public boolean getStatusUp(){
    return m_pivotUp.get();
  }

  public boolean getStatusDown(){
    return m_pivotDown.get();
  }

  public boolean getStatusShoot(){
    return m_pivotShoot.get();
  }
}
