// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.armSubsystem;

/****************CARDENAS APPROVED ****************** */

public class ArmControlUp extends CommandBase {
  /** Creates a new ArmControlUp. */
  private final armSubsystem m_armUp;
  public ArmControlUp(armSubsystem armSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armUp = armSub;
    addRequirements(m_armUp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armUp.armRun(-SpeedConstants.mArmSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armUp.armRun(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_armUp.getStatusUp() == false;
  }
}
