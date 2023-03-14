// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.driveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoGetOnCharge extends SequentialCommandGroup {
  /** Creates a new autoGetOnCharge. */
  public autoGetOnCharge(String direction, double robotSpeed, driveSubsystem drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToPitch(direction, robotSpeed, drive)
        .withTimeout(5),
      new DriveToPitch("Level", 0.4, drive)
        .withTimeout(5)
    );
  }
}