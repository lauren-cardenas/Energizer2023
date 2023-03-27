// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.uClawPivotSubsystem;
import frc.robot.subsystems.uClawRollerSubsystem;

public class uClawPivot extends CommandBase {
  /** Creates a new uClawPivot. */
  private final uClawPivotSubsystem m_pivot;
  private final uClawRollerSubsystem m_roller;
  private final String m_position;

  public uClawPivot(String position, uClawPivotSubsystem pivot, uClawRollerSubsystem roller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivot = pivot;
    m_position = position;
    m_roller = roller;

    addRequirements(pivot, roller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_roller.UclawRollerRun(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_position == "Up"){
      m_pivot.UclawPivotRun(SpeedConstants.mUClawPivotSpeed);
    }
    else {
      m_pivot.UclawPivotRun(-SpeedConstants.mUClawPivotSpeed);
    }

    if(m_position == "Out"){
      m_roller.UclawRollerRun(SpeedConstants.mUclawInSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.UclawPivotRun(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (m_position){
      case "In":
        return m_pivot.getStatusUp() == false;
      case "Shoot":
        return m_pivot.getStatusShoot() == false;
      case "Out":
        return m_pivot.getStatusDown() == false;
      default:
        return false;
    }
  }
}
