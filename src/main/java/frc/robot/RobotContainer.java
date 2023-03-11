// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.ArmControlDown;
import frc.robot.commands.ArmControlUp;
import frc.robot.commands.LiftControl;
import frc.robot.commands.autos.DriveDistanceCommand;
import frc.robot.commands.autos.ScoreCone;
import frc.robot.commands.autos.autoScoreDrive;
import frc.robot.subsystems.UclawSubsystem;
import frc.robot.subsystems.VclawSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.liftSubsystem;
import frc.robot.subsystems.photonvisionSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
// ready for drive only
public class RobotContainer {

  private final driveSubsystem m_robotDrive = new driveSubsystem();
  private final liftSubsystem m_lift = new liftSubsystem();
  private final armSubsystem m_arm = new armSubsystem();
  private final VclawSubsystem m_Vclaw = new VclawSubsystem();
  //private final UclawSubsystem m_Uclaw = new UclawSubsystem();
  private final photonvisionSubsystem m_photon = new photonvisionSubsystem();

  CommandXboxController m_driverController = new CommandXboxController(OIConstants.mDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.mOperatorControllerPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /***AUTO COMMANDS***/

  private final Command m_simpleDeliverAuto =
    new ScoreCone("High", m_lift, m_arm, m_Vclaw);
  
  private final Command m_scoreHighDriveFarAuto =
    new autoScoreDrive("High", AutoConstants.kAutoLongDistance, m_robotDrive, m_lift, m_arm, m_Vclaw);

  private final Command m_scoreHighDriveCloseAuto =
    new autoScoreDrive("High", AutoConstants.kAutoShortDistance, m_robotDrive, m_lift, m_arm, m_Vclaw);
  
  private final Command m_scoreCubeDriveFarAuto =
    new autoScoreDrive("Mid", AutoConstants.kAutoLongDistance, m_robotDrive, m_lift, m_arm, m_Vclaw);

  private final Command m_scoreCubeDriveCloseAuto =
    new autoScoreDrive("Mid", AutoConstants.kAutoShortDistance, m_robotDrive, m_lift, m_arm, m_Vclaw);

  private final Command m_simpleDriveAuto =
    new DriveDistanceCommand(AutoConstants.kAutoShortDistance, AutoConstants.kAutoForwardSpeed, m_robotDrive);
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  
    configureBindings();

    // Driver
    m_robotDrive.setDefaultCommand(
      new RunCommand(() -> m_robotDrive.arcadeDrive(
        ((m_driverController.getRightTriggerAxis()
        - m_driverController.getLeftTriggerAxis())),
        -m_driverController.getLeftX() * SpeedConstants.mturnSpeed
      ), m_robotDrive));

    //Auto Choices
    autoChooser.setDefaultOption("Far_Cone", m_scoreHighDriveFarAuto);
    autoChooser.addOption("Close_Cone", m_scoreHighDriveCloseAuto);
    autoChooser.addOption("Far_Cube", m_scoreCubeDriveFarAuto);
    autoChooser.addOption("Close_Cube", m_scoreCubeDriveCloseAuto);
    autoChooser.addOption("OnlyDeliver", m_simpleDeliverAuto);
    autoChooser.addOption("OnlyDrive", m_simpleDriveAuto);

   
  }


  private void configureBindings() {

    //*******************Driver*******************//

      m_driverController.a()
        .onTrue(Commands.runOnce(() -> m_robotDrive.setMaxSpeed(SpeedConstants.mHalfSpeed), m_robotDrive))
        .onFalse(Commands.runOnce(() -> m_robotDrive.setMaxSpeed(SpeedConstants.driveSpeed), m_robotDrive));

      m_driverController.y()
        .onTrue(Commands.runOnce(() -> m_robotDrive.setMaxSpeed(1.0), m_robotDrive))
        .onFalse(Commands.runOnce(() -> m_robotDrive.setMaxSpeed(SpeedConstants.driveSpeed), m_robotDrive));
    
    /*
     //Uclaw In 'A' Button
     m_driverController.a()
     .onTrue(Commands.runOnce(() -> m_Uclaw.UclawRun(-SpeedConstants.mUclawSpeed), m_Uclaw))
     .onFalse(Commands.runOnce(() -> m_Uclaw.UclawRun(0.0), m_Uclaw));

     //Uclaw Out 'B' Button
     m_driverController.b()
     .onTrue(Commands.runOnce(() -> m_Uclaw.UclawRun(SpeedConstants.mUclawSpeed), m_Uclaw))
     .onFalse(Commands.runOnce(() -> m_Uclaw.UclawRun(0.0), m_Uclaw));
    */

    //*******************Operator*******************//

    //Lift Deliver High 'Y'
      m_operatorController.y()
        .onTrue(new LiftControl("High", m_lift))
        .onFalse(new SequentialCommandGroup(
          new ArmControlUp(m_arm)
            .withTimeout(0.5),
          new LiftControl("In", m_lift)));

    //Lift Deliver Mid 'B'
    
      m_operatorController.b()
        .onTrue(new LiftControl("Mid", m_lift))
        .onFalse(new SequentialCommandGroup(
          new ArmControlUp(m_arm)
            .withTimeout(0.5),
          new LiftControl("In", m_lift)));

    //Lift Retract 'A'
      m_operatorController.a()
        .onTrue(new LiftControl("In", m_lift))
        .onFalse(Commands.runOnce(() -> m_lift.liftRun(0.0), m_lift));

    //Lift Cancel 'X'
      m_operatorController.x()
        .onTrue(Commands.runOnce(() -> m_lift.liftRun(0.0), m_lift))
        .onFalse(Commands.runOnce(() -> m_lift.liftRun(0.0), m_lift));
    
    //Arm Raise 'Right Bumper'
      m_operatorController.rightBumper()
        .onTrue(new ArmControlUp(m_arm))
        .onFalse(Commands.runOnce(() -> m_arm.armRun(0.0), m_arm));

    //Arm Lower 'Right Trigger'
      m_operatorController.rightTrigger()
        .onTrue(new ArmControlDown(m_arm))
        .onFalse(Commands.runOnce(() -> m_arm.armRun(0.0), m_arm));
    
    //Vclaw In 'DownDPad' Button
      m_operatorController.povDown() 
      .onTrue(Commands.runOnce(() -> m_Vclaw.VclawRun(SpeedConstants.mVclawSpeed), m_Vclaw))
      .onFalse(Commands.runOnce(() -> m_Vclaw.VclawRun (0.1), m_Vclaw));

    //Vclaw Out 'UpDPad' Button
      m_operatorController.povUp() 
      .onTrue(Commands.runOnce(() -> m_Vclaw.VclawRun(-SpeedConstants.mVclawSpitSpeed), m_Vclaw))
      .onFalse(Commands.runOnce(() -> m_Vclaw.VclawRun(0.0), m_Vclaw));
    
  }

 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
