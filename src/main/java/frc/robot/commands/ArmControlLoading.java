// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.armSubsystem;

public class ArmControlLoading extends CommandBase {
  /** Creates a new ArmControlLoading. */
  private final armSubsystem m_arm;


  public ArmControlLoading(armSubsystem armSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = armSub;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.armRun(0.85); //initializes by setting arm speed down
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.getStatusDown() == false){
      m_arm.armRun(-SpeedConstants.mArmSpeed); //changes arm speed to up if down hit
    } else if(m_arm.getStatusUp() == false){
      m_arm.armRun(0.7); //changes arm speed to down if top hit
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.armRun(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.getStatusLoading() == false;
  }
}
