// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Spin extends CommandBase {
  /** Creates a new Spin. */
  DriveSubsystem m_drive;
  double initial=0;
  double destination;
  public Spin(DriveSubsystem drive, double rads) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_drive=drive;
    destination=rads;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initial = m_drive.getYaw()/360*2*Math.PI;
    destination+=initial;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(destination-initial<0){
      m_drive.drive(0,0,-1,false);
    }
    else{
      m_drive.drive(0,0,1,false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0,0,0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drive.getYaw()/180*Math.PI-destination)<0.1;
  
  }
}
