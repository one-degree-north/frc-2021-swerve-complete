// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final CANCoder m_turningEncoder;

  private final CANEncoder m_driveEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel,int turningMotorChannel, int CAN, double angleOff) {

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    this.m_turningEncoder = new CANCoder(CAN);
    this.m_driveEncoder= m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(Math.PI*Constants.ModuleConstants.kWheelDiameterMeters);
    //m_turningEncoder.configFeedbackCoefficient(0.087890625, "radians", SensorTimeBase.PerSecond);
    resetEncoders(angleOff);
    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getAngle()));
  }

  public double getAngle(){
      double raw = m_turningEncoder.getPosition()/180*Math.PI;
      
      
      return raw;
  }

  
  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  double numRot=0;
  double prevAngle;
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    if(Math.abs(prevAngle-(state.angle.getRadians()+numRot))>Math.PI){
      if(prevAngle>state.angle.getRadians()+numRot){
        numRot+=2*Math.PI;
      }
      else{
        numRot-=2*Math.PI;
      }
    }
      

    final var driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turningPIDController.calculate(getAngle(), state.angle.getRadians());

    //System.out.println("position " + m_driveEncoder.getPosition() + " drive " + m_driveEncoder.getVelocity()/1000 + "m/s State " + state.speedMetersPerSecond  + "Current" + driveOutput);
    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(state.speedMetersPerSecond/AutoConstants.kMaxSpeedMetersPerSecond*0.2);
    m_turningMotor.set((state.angle.getRadians()+numRot-getAngle())/(2*Math.PI));
    prevAngle=state.angle.getRadians()+numRot;
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders(double angOff) {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(m_turningEncoder.getAbsolutePosition()%360-angOff);
  }

  public void resetEncoder() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
  }
 
}
