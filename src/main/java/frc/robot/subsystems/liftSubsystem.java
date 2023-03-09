// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MechConstants;

public class liftSubsystem extends SubsystemBase {
  /** Creates a new liftSubsystem. */
  WPI_VictorSPX m_lift;

  private DigitalInput liftSwitchIn = new DigitalInput(MechConstants.m_liftInSwitch);
  private DigitalInput liftSwitchMid = new DigitalInput(MechConstants.m_liftMidSwitch);
  private DigitalInput liftSwitchHigh = new DigitalInput(MechConstants.m_liftHighSwitch);

  public liftSubsystem() {
    m_lift = new WPI_VictorSPX(MechConstants.m_liftPort);
  }

  @Override
  public void periodic() {
  }

  public void liftRun(double speed){
    m_lift.set(-speed);
  }

  public void liftReverse(double speed){
    m_lift.set(speed);
  }

  public boolean getStatusIn(){
    return liftSwitchIn.get();
  }

  public boolean getStatusMid(){
    return liftSwitchMid.get();
  }

  public boolean getStatusHigh(){
    return liftSwitchHigh.get();
  }

  public void displayLiftLimits(){
    SmartDashboard.putBoolean("Limit 0", getStatusHigh());
    SmartDashboard.putBoolean("1", getStatusMid());
    SmartDashboard.putBoolean("2", getStatusIn());
  }
}

