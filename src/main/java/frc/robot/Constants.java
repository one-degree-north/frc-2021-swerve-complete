// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 4;
    public static final int kRearLeftDriveMotorPort = 8;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kRearRightDriveMotorPort = 6;

    public static final int kFrontLeftTurningMotorPort = 3;
    public static final int kRearLeftTurningMotorPort = 7;
    public static final int kFrontRightTurningMotorPort = 1;
    public static final int kRearRightTurningMotorPort = 5;

    public static final int kFrontLeftEncoderPort = 9;
    public static final int kRearLeftEncoderPort = 10;
    public static final int kFrontRightEncoderPort = 12;
    public static final int kRearRightEncoderPort = 11;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 18.5*2.54/100;
    // Distance between front and back wheels on robot
    public static final double kWheelBase = 18.5*2.54/100;

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    public static final double kMaxSpeedMetersPerSecond = 3.38;
  }

  public static final class ModuleConstants {
    public static final double kWheelDiameterMeters = 4*2.54/100;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = 16*Math.PI;//used to be Pi/4
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 256*Math.PI*Math.PI;//used to be pi/4

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;//used to be 10

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
