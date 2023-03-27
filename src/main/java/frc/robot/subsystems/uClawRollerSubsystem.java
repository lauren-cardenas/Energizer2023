// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class uClawRollerSubsystem extends SubsystemBase {
  /** Creates a new CclawSubsystem. */
  CANSparkMax m_Uclaw;

  public uClawRollerSubsystem() {
    m_Uclaw = new CANSparkMax(MechConstants.m_uClawRollerPort, MotorType.kBrushless);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void UclawRollerRun(double speed){
    m_Uclaw.set(speed);
  }
  
}
