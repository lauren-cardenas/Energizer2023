// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 // ready for drive only
public final class Constants {
  public static final class OIConstants {
    public static final int mDriverControllerPort = 0;
    public static final int mOperatorControllerPort = 1;
}

  public static final class DriveConstants{
    public static final int mLeftDrivePort1 = 1;
    public static final int mLeftDrivePort2 = 2;
    public static final int mRightDrivePort1 = 3;
    public static final int mRightDrivePort2 = 4;

    public static final double mTrackwidthMeters = 0.6;
    public static final DifferentialDriveKinematics kDriveKinematics = 
      new DifferentialDriveKinematics(mTrackwidthMeters);

    public static final double mGearRatio = 10.93;
    public static final int mEncoderCPR = 2048;
    public static final double mWheelDiameterMeters = 0.1524;
    public static final double mEncoderDistancePerPulse = 
      (mWheelDiameterMeters * Math.PI) /mGearRatio/ (double) mEncoderCPR;

    public static final double Conversion = 
      mWheelDiameterMeters * Math.PI /
          (mGearRatio * mEncoderCPR);

    public static final double kRampTime = 0.1;

    public static final double kCurrentLimit = 30;
    public static final double kCurrentThreshold = 40;
    public static final double kLimitTime = 1;
    public static final boolean kSupplyEnable = true;
    public static final boolean kStatorEnable = false;
    public static final double kStatorLimit = 50;
    public static final double kStatorThreshold = 60;
    public static final double kStatorTime = 1;

  }

  public static final class AutoConstants{
    public static final double kAutoTurnSpeed = 0.45;
    public static final double kAutoForwardSpeed = -0.5;
    public static final double kAutoReverseSpeed = 0.5;

    public static final double kAutoLongDistance = 4;
    public static final double kAutoShortDistance = 2.5;

    public static final double kLevelAngle = -3;
    public static final double kReverseClimbAngle = -10.0;
    public static final double kForwardClimbAngle = 10.0;
  }
  public static final class SpeedConstants{
    public static final double mturnSpeed = 0.7; //0.6;
    public static final double driveSpeed = 1;
    public static final double mHalfSpeed = 0.5;

    public static final double mLiftSpeed = 0.8; //0.6
    public static final double mLiftDownSpeed = 0.4;
    public static final double mArmSpeed = 1.0;
    public static final double mVclawSpeed = 0.8;
    public static final double mVclawSpitSpeed = 0.25;
    public static final double mVclawCubeSpeed = 0.9;
    public static final double mUclawInSpeed = 0.75;
    public static final double mUclawSpitSpeed = 0.8; //.5 for high 
    public static final double mUclawSpitMidSpeed = 0.5; // .3 for mid
    public static final double mUClawPivotSpeed = 0.468;
    public static final double mUClawPivotDownSpeed = 0.3;
    public static final double mUClawPivotShootSpeed = 0.17;
  }
  public static final class MechConstants{
    // lift port 
    public static final int m_liftPort = 5;
    // Arm port
    public static final int m_armPort = 6; 
    // Vclaw port
    public static final int m_VclawPort = 7;
    //Cclaw port
    public static final int m_UclawPort = 8;
    //Uclaw roller port
    public static final int m_uClawRollerPort = 3;
    //switches
    public static final int mArmDownSwitch = 5; 
    public static final int m_armSwitchUp = 3; 
    public static final int m_liftHighSwitch = 0; 
    public static final int m_liftMidSwitch = 1;  
    public static final int m_liftInSwitch = 2;
    public static final int m_loadingSwitch = 6;
    public static final int m_pivotDownSwitch = 8;
    public static final int m_pivotUpSwitch = 7;
    public static final int m_pivotShootSwitch = 9;

  }


  }

