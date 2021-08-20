// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.TrajectoryCommand;
import frc.robot.subsystems.DriveSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  //limelight
  NetworkTable table;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -m_driverController.getY(GenericHID.Hand.kLeft),
                    -m_driverController.getX(GenericHID.Hand.kLeft),
                    -m_driverController.getX(GenericHID.Hand.kRight),
                    false), m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    JoystickButton button = new JoystickButton(m_driverController, XboxController.Button.kA.value);
    Command target = new RunCommand(()->{
        double targetOffsetAngle_Horizontal = -table.getEntry("tx").getDouble(0);
        //double targetOffsetAngle_Vertical = -table.getEntry("ty").getDouble(0);
        System.out.println(table.getEntry("tx"));
        if(Math.abs(targetOffsetAngle_Horizontal)<1){
            targetOffsetAngle_Horizontal=0;
        }
        m_robotDrive.drive(0, 0, -targetOffsetAngle_Horizontal/2, false);
    }, m_robotDrive);
    button.toggleWhenPressed(target);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    m_robotDrive.resetAllEncoders();
    

   TrajectoryCommand traj = new TrajectoryCommand(m_robotDrive, List.of(), new Pose2d(0.9, 0, new Rotation2d(Math.PI/2)));
   //Spin spin = new Spin(m_robotDrive, 3*Math.PI);
   //TrajectoryCommand rot = new TrajectoryCommand(m_robotDrive, List.of(), new Pose2d(0, 0, new Rotation2d(0)));
    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(
        traj
        // spin,
        // new Spin(m_robotDrive, -Math.PI/4),
        // new Spin(m_robotDrive, Math.PI/2),
        // new Spin(m_robotDrive, -Math.PI/2),
        // new Spin(m_robotDrive, Math.PI/4),
        // new TrajectoryCommand(m_robotDrive, List.of(), new Pose2d(0.5,0, new Rotation2d(0)))

    );
  }
}