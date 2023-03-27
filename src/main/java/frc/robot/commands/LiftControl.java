// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SpeedConstants;
import frc.robot.subsystems.liftSubsystem;

/*************For this Command, use "High", "Mid", and "In" for switchport************* */
/*************See RobotContainer for proper use******************** */

public class LiftControl extends CommandBase {
  /** Creates a new LiftControl. */

  private final liftSubsystem m_lift;
  private final String m_switch;
  private final double m_speed;

  public LiftControl(String position, double speed, liftSubsystem lift) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    m_switch = position;
    m_speed = speed;

    addRequirements(lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_switch == "In"){
      m_lift.liftRun(SpeedConstants.mLiftDownSpeed);
    } else{
      m_lift.liftRun(-m_speed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lift.liftRun(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    switch (m_switch) {
      case "High":
        return m_lift.getStatusHigh() == false;
      case "Mid":
        return m_lift.getStatusMid() == false;
      case "In":
        return m_lift.getStatusIn() == false;
      default:
        return false;
    }
  }
}
