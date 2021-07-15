// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final CANCoder m_turningEncoder;

  private final CANEncoder m_driveEncoder;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveCAN,int turningCAN, int encoderCAN, double angleOff) {

    m_driveMotor = new CANSparkMax(driveCAN, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningCAN, MotorType.kBrushless);

    this.m_turningEncoder = new CANCoder(encoderCAN);
    this.m_driveEncoder= m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(Math.PI*Constants.ModuleConstants.kWheelDiameterMeters);
    m_driveEncoder.setVelocityConversionFactor(Math.PI/360*Constants.ModuleConstants.kWheelDiameterMeters);
    resetEncoders(angleOff);
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
    return m_turningEncoder.getPosition()/180*Math.PI;
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

    m_driveMotor.set(state.speedMetersPerSecond/AutoConstants.kMaxSpeedMetersPerSecond*0.6);
    m_turningMotor.set((state.angle.getRadians()+numRot-getAngle())/(2*Math.PI));
    prevAngle=state.angle.getRadians()+numRot;
    //System.out.println(m_driveEncoder.getVelocity());
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders(double angOff) {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(m_turningEncoder.getAbsolutePosition()%360-angOff);
  }
 
}
