// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.ArmControlUp;
import frc.robot.commands.LiftControl;
import frc.robot.subsystems.VclawSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.liftSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoGetOnCharge extends SequentialCommandGroup {
  /** Creates a new autoGetOnCharge. */
  public autoGetOnCharge(String direction, double robotSpeed, driveSubsystem drive,
                        liftSubsystem lift, armSubsystem arm, VclawSubsystem vclaw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreCone("High", lift, arm, vclaw),
      new DriveToPitch(direction, robotSpeed, drive)
        .alongWith(new LiftControl("In", SpeedConstants.mLiftDownSpeed, lift))
        .withTimeout(10),
      new DriveToPitch("LevelB", 0.4, drive)
        .withTimeout(10),
      new WaitCommand(0.05),
      new DriveToPitch("LevelA", -0.4, drive)
        .withTimeout(10)
      
    );
  }
}
