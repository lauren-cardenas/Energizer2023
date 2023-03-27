// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.ArmControlLoading;
import frc.robot.commands.LiftControl;
import frc.robot.subsystems.VclawSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.liftSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoScoreDrive extends SequentialCommandGroup {
  /** Creates a new autoScoreDrive. */
  public autoScoreDrive(String position, double distance,
                        driveSubsystem drive, liftSubsystem lift,
                        armSubsystem arm, VclawSubsystem vclaw) {

    addCommands(
      new ScoreCone(position, lift, arm, vclaw), //score game piece
      new DriveDistanceCommand(-distance, AutoConstants.kAutoReverseSpeed, drive)
        .alongWith(new LiftControl("In", SpeedConstants.mLiftDownSpeed, lift)) //drive backwards
        .raceWith(Commands.waitSeconds(10)), //times out at 3 seconds
      Commands.runOnce(() -> drive.arcadeDrive(0, 0), drive),
      new turnSimple(drive, 150, true)
        .alongWith(new ArmControlLoading(arm))
    );
  }
}
