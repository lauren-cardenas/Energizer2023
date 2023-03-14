// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.ArmControlDown;
import frc.robot.commands.ArmControlUp;
import frc.robot.commands.LiftControl;
import frc.robot.subsystems.VclawSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.liftSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreCone extends SequentialCommandGroup {
  /** Creates a new ScoreConeLong. */
  public ScoreCone(String liftPosition, liftSubsystem lift,
                        armSubsystem arm, VclawSubsystem Vclaw) {

    addCommands(
      new ArmControlDown(arm)
        .alongWith(new LiftControl(liftPosition, lift)),
      Commands.waitSeconds(0.5),
      Commands.runOnce(() -> Vclaw.VclawRun(-SpeedConstants.mVclawSpitSpeed), Vclaw),
      Commands.waitSeconds(1),
      new ArmControlUp(arm)
        .alongWith(Commands.runOnce(() -> Vclaw.VclawRun(0.0), Vclaw))
        .alongWith(new LiftControl("In", lift)).raceWith(Commands.waitSeconds(0.5))
        
    );
  }
}
