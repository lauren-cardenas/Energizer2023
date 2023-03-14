// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SpeedConstants;

public class driveSubsystem extends SubsystemBase {
  /** Creates a new driveSubsystem. */
  private final WPI_TalonFX m_frontLeft = new WPI_TalonFX(DriveConstants.mLeftDrivePort1);
  private final WPI_TalonFX m_backLeft = new WPI_TalonFX(DriveConstants.mLeftDrivePort2);
  private final WPI_TalonFX m_frontRight = new WPI_TalonFX(DriveConstants.mRightDrivePort1);
  private final WPI_TalonFX m_backRight = new WPI_TalonFX(DriveConstants.mRightDrivePort2);

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_frontLeft, m_backLeft);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_frontRight, m_backRight);

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  
   //drive
  public driveSubsystem() {
    m_rightMotors.setInverted(true);

    m_drive.setSafetyEnabled(false);

    m_drive.setMaxOutput(SpeedConstants.driveSpeed);

    m_gyro.reset();
    

    m_frontLeft.configFactoryDefault();
    m_frontRight.configFactoryDefault();
    m_backLeft.configFactoryDefault();
    m_backRight.configFactoryDefault();

    /********CARDENAS EXPERIMENTATIONS*************/
    //ramp rate
    m_frontLeft.configOpenloopRamp(DriveConstants.kRampTime);
    m_frontRight.configOpenloopRamp(DriveConstants.kRampTime);
    m_backLeft.configOpenloopRamp(DriveConstants.kRampTime);
    m_backRight.configOpenloopRamp(DriveConstants.kRampTime);

    m_frontLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,40,45,2));
    m_frontRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,40,45,2));
    m_backLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,40,45,2));
    m_backRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,40,45,2));


    //stator limit
    m_frontLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,45,2));
    m_frontRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,45,2));
    m_backLeft.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,45,2));
    m_backRight.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40,45,2));


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void arcadeDrive(double fwd, double rot){
    m_drive.arcadeDrive(fwd, rot);
    displayEncoderValues();
    displayGyroPitch();
  }

  public void tankDrive(double left, double right){
    m_drive.tankDrive(left, right);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void resetEncoders() {
    m_frontLeft.setSelectedSensorPosition(0);
    m_frontRight.setSelectedSensorPosition(0);
  }

  public double getAverageEncoderDistance() {
    return (getLeftWheelPosition() + getRightWheelPosition()) / 2;
  }

  public void displayEncoderValues() {
    SmartDashboard.putNumber("Right Data", getRightWheelPosition());
    SmartDashboard.putNumber("Left Data", getLeftWheelPosition());
  }

  public void displayGyroPitch(){
    SmartDashboard.putNumber("Pitch Angle", m_gyro.getRoll());
  }

  private double getLeftWheelPosition() {
    return (m_frontLeft.getSelectedSensorPosition() * DriveConstants.mWheelDiameterMeters * Math.PI
    / DriveConstants.mEncoderCPR) / DriveConstants.mGearRatio;
  }

  private double getRightWheelPosition() {
    return (-m_frontRight.getSelectedSensorPosition() * DriveConstants.mWheelDiameterMeters * Math.PI
    / DriveConstants.mEncoderCPR) / DriveConstants.mGearRatio;
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getPitch(){
    return m_gyro.getRoll();
  }

  public void setMaxSpeed(double speed){
    m_drive.setMaxOutput(speed);
  }

  
}
