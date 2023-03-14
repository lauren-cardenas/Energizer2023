// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.driveSubsystem;

public class DriveToPitch extends CommandBase {
  /** Creates a new DriveToPitch. */
  private final driveSubsystem m_drive;
  private final double m_speed;
  private final String m_direction;

  public DriveToPitch(String direction, double robotSpeed, driveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speed = robotSpeed;
    m_drive = drive;
    m_direction = direction;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(-m_speed, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    switch (m_direction) {
      case "Level":
        return (m_drive.getPitch()) >= AutoConstants.kLevelAngle;
      case "Reverse":
        return (m_drive.getPitch()) <= AutoConstants.kReverseClimbAngle;
      case "Forward":
        return (m_drive.getPitch()) >= AutoConstants.kForwardClimbAngle;
      default:
        return false;
    }
    // if(m_direction == "level"){
    //   return (m_drive.getPitch()) <= AutoConstants.kLevelAngle;
    // }else{
    //   return (m_drive.getPitch()) <= m_angle;
    // }
  }
}
