// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class StraightCommand extends CommandBase {
  /** Creates a new Straight. */

  DriveSubsystem m_drive;
  double m_angle;
  double m_distance;
  Translation2d m_translational;
  public StraightCommand(double distance, double angle,  DriveSubsystem drivesys) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivesys);
    this.m_drive=drivesys;
    this.m_angle=angle;
    this.m_distance=distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_translational=m_drive.getTranslations();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(Math.cos(Math.toRadians(m_angle)),Math.sin(Math.toRadians(m_angle)),0,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0,0,0,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_drive.getTranslations().getDistance(m_translational)>m_distance);
  }
}
