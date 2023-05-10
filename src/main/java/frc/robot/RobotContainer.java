// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SpeedConstants;
import frc.robot.commands.ArmControlDown;
import frc.robot.commands.ArmControlLoading;
import frc.robot.commands.ArmControlUp;
import frc.robot.commands.LiftControl;
import frc.robot.commands.uClawPivot;
import frc.robot.commands.autos.DriveDistanceCommand;
import frc.robot.commands.autos.ScoreCone;
import frc.robot.commands.autos.autoGetOnCharge;
import frc.robot.commands.autos.autoScoreDrive;
import frc.robot.subsystems.VclawSubsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.driveSubsystem;
import frc.robot.subsystems.liftSubsystem;
import frc.robot.subsystems.photonvisionSubsystem;
import frc.robot.subsystems.uClawPivotSubsystem;
import frc.robot.subsystems.uClawRollerSubsystem;


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
  private final uClawPivotSubsystem m_uClawPivot = new uClawPivotSubsystem();
  private final uClawRollerSubsystem m_UClawRoller = new uClawRollerSubsystem();
  private final photonvisionSubsystem m_photon = new photonvisionSubsystem();

  CommandXboxController m_driverController = new CommandXboxController(OIConstants.mDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.mOperatorControllerPort);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /***AUTO COMMANDS***/

  private final SequentialCommandGroup m_scoreHighPickAuto =
    new autoScoreDrive("High", AutoConstants.kAutoLongDistance, m_robotDrive, m_lift, m_arm, m_Vclaw).andThen(
    new uClawPivot("Out", m_uClawPivot, m_UClawRoller));

  private final SequentialCommandGroup m_ScoreAndWait =
    new ScoreCone("High", m_lift, m_arm, m_Vclaw).andThen(
      new WaitCommand(7)).andThen(
      new DriveDistanceCommand(-AutoConstants.kAutoLongDistance, 0.6, m_robotDrive).alongWith(
        new LiftControl("In", SpeedConstants.mLiftDownSpeed, m_lift))
      );

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

  private final Command m_scoreMidDriveFarAuto =
    new autoScoreDrive("Mid", AutoConstants.kAutoLongDistance, m_robotDrive, m_lift, m_arm, m_Vclaw);

  private final Command m_scoreMidDriveCloseAuto =
    new autoScoreDrive("Mid", AutoConstants.kAutoShortDistance, m_robotDrive, m_lift, m_arm, m_Vclaw);

  private final Command m_scoreHighChargeCone =
    new autoGetOnCharge("Reverse", 0.5, m_robotDrive, m_lift, m_arm, m_Vclaw);
 
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
    autoChooser.addOption("Mid_Far_Cone", m_scoreMidDriveFarAuto);
    autoChooser.addOption("Mid_Close_Cone", m_scoreMidDriveCloseAuto);
    autoChooser.addOption("OnlyDeliver", m_simpleDeliverAuto);
    autoChooser.addOption("OnlyDrive", m_simpleDriveAuto);
    autoChooser.addOption("Far and Pick", m_scoreHighPickAuto);
    autoChooser.addOption("Score and Wait", m_ScoreAndWait);
    autoChooser.addOption("High_Cone_Chargeh", m_scoreHighChargeCone);

    SmartDashboard.putData("Autonomous", autoChooser);

   
  }


  private void configureBindings() {

    //*******************Driver*******************//

    
    //U Claw Down 'X button'
      m_operatorController.x()
        .onTrue(new uClawPivot("Out", m_uClawPivot, m_UClawRoller))
        .onFalse(new uClawPivot("In", m_uClawPivot, m_UClawRoller));
    
    //U Claw Aim 'Left Trigger'
      m_operatorController.leftTrigger()
        .onTrue(new uClawPivot("Shoot", m_uClawPivot, m_UClawRoller))
        .onFalse(new uClawPivot("In", m_uClawPivot, m_UClawRoller));

    //U Claw Shoot High 'start button'
      m_operatorController.start()
        .onTrue(new RunCommand(() -> m_UClawRoller.UclawRollerRun(SpeedConstants.mUclawSpitSpeed), m_UClawRoller))  
      //.onTrue(Commands.runOnce(() -> m_UClawRoller.UclawRollerRun(SpeedConstants.mUclawSpitSpeed), m_UClawRoller))
        .onFalse(Commands.runOnce(() -> m_UClawRoller.UclawRollerRun(0.0), m_UClawRoller));

    //U Claw Shoot High 'right stick button'
    m_operatorController.rightStick()
    .onTrue(new RunCommand(() -> m_UClawRoller.UclawRollerRun(SpeedConstants.mUclawSpitSpeed), m_UClawRoller))  
  //.onTrue(Commands.runOnce(() -> m_UClawRoller.UclawRollerRun(SpeedConstants.mUclawSpitSpeed), m_UClawRoller))
    .onFalse(Commands.runOnce(() -> m_UClawRoller.UclawRollerRun(0.0), m_UClawRoller));
    
    //U Claw Shoot Mid 'left stickk button'
    m_operatorController.leftStick()
    .onTrue(new RunCommand(() -> m_UClawRoller.UclawRollerRun(SpeedConstants.mUclawSpitMidSpeed), m_UClawRoller))
    .onFalse(Commands.runOnce(() -> m_UClawRoller.UclawRollerRun(0.0), m_UClawRoller));

     //U Claw Shoot Mid 'back button'
     m_operatorController.back()
     .onTrue(new RunCommand(() -> m_UClawRoller.UclawRollerRun(SpeedConstants.mUclawSpitMidSpeed), m_UClawRoller))
     .onFalse(Commands.runOnce(() -> m_UClawRoller.UclawRollerRun(0.0), m_UClawRoller));


    /* Seniora Cardeenas Code
    U Claw Roller Nudge
    m_UClawRoller.setDefaultCommand(new RunCommand(() -> m_UClawRoller.UclawRollerRun(m_driverController.getRightY() * -SpeedConstants.mUclawInSpeed), m_UClawRoller));
     m_driverController.rightBumper()
     .onTrue(Commands.runOnce(() -> m_UClawRoller.UclawRollerRun(-SpeedConstants.mUclawInSpeed), m_UClawRoller))
     .onFalse(Commands.runOnce(() -> m_UClawRoller.UclawRollerRun(0), m_UClawRoller));
    */

    m_UClawRoller.setDefaultCommand(new RunCommand(() -> m_UClawRoller.UclawRollerRun(m_operatorController.getRightY()* -SpeedConstants.mUclawInSpeed), m_UClawRoller));
    
    //*******************Operator*******************//

    //Lift
      m_lift.setDefaultCommand(new RunCommand(() -> m_lift.liftRun(m_operatorController.getLeftY() * SpeedConstants.mLiftSpeed), m_lift));

    //Lift Deliver High 'Y'
      m_operatorController.y()
        .onTrue(new LiftControl("High", SpeedConstants.mLiftSpeed, m_lift))
        .onFalse(new SequentialCommandGroup(
          new ArmControlUp(m_arm)
            .withTimeout(0.5),
          new LiftControl("In", SpeedConstants.mLiftDownSpeed, m_lift)));

    //Lift Deliver Mid 'B'
    
      m_operatorController.b()
        .onTrue(new LiftControl("Mid", SpeedConstants.mLiftSpeed, m_lift))
        .onFalse(new SequentialCommandGroup(
          new ArmControlUp(m_arm)
            .withTimeout(0.5),
          new LiftControl("In", SpeedConstants.mLiftDownSpeed, m_lift)));

    //Lift Retract 'A'
      m_operatorController.a()
        .onTrue(new LiftControl("In", SpeedConstants.mLiftDownSpeed, m_lift))
        .onFalse(Commands.runOnce(() -> m_lift.liftRun(0.0), m_lift));
    
    //Arm Raise 'Right Bumper'
      m_operatorController.rightBumper()
        .onTrue(new ArmControlUp(m_arm))
        .onFalse(Commands.runOnce(() -> m_arm.armRun(0.0), m_arm));

    //Arm set to Loading Station 'Left bumper'
      m_operatorController.leftBumper()
        .onTrue(new ArmControlLoading(m_arm))
        .onFalse(Commands.runOnce(() -> m_arm.armRun(0.0), m_arm));

    //Arm Lower 'Right Trigger'
      m_operatorController.rightTrigger()
        .onTrue(new ArmControlDown(m_arm))
        .onFalse(Commands.runOnce(() -> m_arm.armRun(0.0), m_arm));
    
    //Vclaw In 'DownDPad' Button
      m_operatorController.povDown() 
      .onTrue(Commands.runOnce(() -> m_Vclaw.VclawRun(SpeedConstants.mVclawSpeed), m_Vclaw))
      .onFalse(Commands.runOnce(() -> m_Vclaw.VclawRun (0.1), m_Vclaw));

    //Vclaw Out -- Cones 'UpDPad' Button
      m_operatorController.povUp() 
      .onTrue(Commands.runOnce(() -> m_Vclaw.VclawRun(-SpeedConstants.mVclawSpitSpeed), m_Vclaw))
      .onFalse(Commands.runOnce(() -> m_Vclaw.VclawRun(0.0), m_Vclaw));

    //Vclaw Out -- Cubes 'RightDPad' Button
      m_operatorController.povRight() 
      .onTrue(Commands.runOnce(() -> m_Vclaw.VclawRun(-SpeedConstants.mVclawCubeSpeed), m_Vclaw))
      .onFalse(Commands.runOnce(() -> m_Vclaw.VclawRun(0.0), m_Vclaw));
    
  }

 
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}





























































































///doo doo feces