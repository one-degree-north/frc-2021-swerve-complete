// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;


import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("PMD.ExcessiveImports")
public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort, 
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftEncoderPort,-20.83);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
        DriveConstants.kRearLeftDriveMotorPort, 
        DriveConstants.kRearLeftTurningMotorPort,
        DriveConstants.kRearLeftEncoderPort,72.510);
      
  private final SwerveModule m_frontRight =
      new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort, 
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightEncoderPort,95.098);
      
  private final SwerveModule m_rearRight =
      new SwerveModule(
        DriveConstants.kRearRightDriveMotorPort, 
        DriveConstants.kRearRightTurningMotorPort,
        DriveConstants.kRearRightEncoderPort,48.076);

  // The gyro sensor
  private final PigeonIMU m_gyro = new PigeonIMU(13);

  // Odometry class for tracking robot pose
  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d());

  public double getYaw(){
    double[] pidge = new double[3];
    m_gyro.getYawPitchRoll(pidge);
    return pidge[0];
  }

  public Rotation2d getRotation2d(){
      return new Rotation2d(getYaw()/360*2*Math.PI);
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    
  }

  public void resetAllEncoders(){
    m_frontLeft.resetEncoders(-20.83);
    m_rearLeft.resetEncoders(72.510);
    m_frontRight.resetEncoders(95.098);
    m_rearRight.resetEncoders(48.076);
 
  }

  public Translation2d getTranslations(){
    return m_odometry.getPoseMeters().getTranslation().times(2.54/100);
  }

  @Override
  public void periodic(){
    // Update the odometry in the periodic block
    m_odometry.update(
        getRotation2d(),
        m_frontLeft.getState(),
        m_rearLeft.getState(),
        m_frontRight.getState(),
        m_rearRight.getState());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
