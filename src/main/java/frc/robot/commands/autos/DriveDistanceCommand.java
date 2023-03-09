// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveSubsystem;

public class DriveDistanceCommand extends CommandBase {
  /** Creates a new DriveDistance. */
  private final driveSubsystem m_drive;
  private final double m_distance;
  private final double m_speed;

  public DriveDistanceCommand(double inches, double speed, driveSubsystem drive) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(-m_speed, 0); //FIXME why negative speed
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.signum(m_distance) == 1){
      return Math.abs(m_drive.getAverageEncoderDistance()) >= m_distance;
    }else{
      return (m_drive.getAverageEncoderDistance()) <= m_distance;
    }
  }
}
