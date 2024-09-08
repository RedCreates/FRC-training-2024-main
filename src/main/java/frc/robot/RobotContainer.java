// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  /**
   */

    private final DriveSubsystem robotDrive = new DriveSubsystem();

    PS4Controller driverController  = new PS4Controller(OIConstants.kDriverControllerPort);

    public RobotContainer() {
          // Configure the trigger bindings
      configureBindings();

      robotDrive.setDefaultCommand(
        new RunCommand(
          () -> robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY()*0.5, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX()*0.5, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRawAxis(4)*0.5, OIConstants.kDriveDeadband),
                true, true), robotDrive)
        );
    }

  

  private void configureBindings() {
    new JoystickButton(driverController, Button.kR1.value).whileTrue(new RunCommand(
      () -> robotDrive.setX(), robotDrive
    ));

    new JoystickButton(driverController, Button.kCircle.value).onTrue(new InstantCommand(
      () -> robotDrive.zeroHeading(), robotDrive
    ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
