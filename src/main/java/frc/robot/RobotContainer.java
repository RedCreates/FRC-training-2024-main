// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.commands.DefaultDriveCommand;
import frc.commands.DefaultTestMotorCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TestMotorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TestMotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

public class RobotContainer {

  private final DriveSubsystem robotDrive = new DriveSubsystem();

  private final TestMotorSubsystem testMotor = new TestMotorSubsystem();

  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);

  XboxController subController = new XboxController(OIConstants.kCoPilotControllerPort);

  CommandXboxController subControllerCommand = new CommandXboxController(OIConstants.kCoPilotControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    setUpShuffleBoard();

    robotDrive.setDefaultCommand(new DefaultDriveCommand(robotDrive));
    testMotor.setDefaultCommand(new DefaultTestMotorCommand(testMotor, TestMotorConstants.kTestMotorSpeedMedium));
  }

  private void configureBindings() {

    setUpShuffleBoard();

    subControllerCommand.y().whileTrue(new RunCommand(() -> testMotor.setPositionalMotorPosition(TestMotorConstants.kTestMotorTopPosition)));

    new JoystickButton(driverController, Button.kX.value).whileTrue(new RunCommand(
        () -> robotDrive.setX(), robotDrive));

    new JoystickButton(driverController, Button.kY.value).onTrue(new InstantCommand(
        () -> robotDrive.zeroHeading(), robotDrive));
    
    new JoystickButton(subController, Button.kX.value).whileTrue(new RunCommand(() -> testMotor.setRotationalMotorDutyCycle(1)));
    new JoystickButton(subController, Button.kB.value).whileTrue(new RunCommand(() -> testMotor.setRotationalMotorDutyCycle(-1)));

  }

  public Command getAutonomousCommand() {
    robotDrive.zeroHeading();

    SendableChooser<autoC> auto = (SendableChooser<autoC>) SmartDashboard.getData("Auto Type");
    var choice = auto.getSelected();
     switch (choice) {
      case Command_1: return Command_1();    
      case Command_2: return Command_2();
      case Command_3: return Command_2();
      case Command_4: return Command_2();
      case Command_5: return Command_2();
      case Command_6: return Command_2();
      case Command_7: return Command_2();
      case Command_8: return Command_2();
      case Command_9: return Command_2();
      default: return command_10();
    }    
    // // Create config for trajectory; basically setting for the trajectory
    // TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSec,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // /*
    //  * Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //  * // Start at the origin facing the +X direction
    //  * new Pose2d(0, 0, new Rotation2d(0)),
    //  * // Pass through these two interior waypoints, making an 's' curve path
    //  * List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //  * // End 3 meters straight ahead of where we started, facing forward
    //  * new Pose2d(3, 0, new Rotation2d(0)),
    //  * config);
    //  */

    // // generate the trajectory
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     List.of( // trajectory motion position
    //         new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)), // final trajectory position
    //         new Pose2d(new Translation2d(1, 0), Rotation2d.fromDegrees(45)),
    //         new Pose2d(new Translation2d(1, -1), Rotation2d.fromDegrees(135)),
    //         new Pose2d(new Translation2d(2, -1), Rotation2d.fromDegrees(180))),
    //     config); // apply the previously written settings

    // // PID Controllers for tracking trjectory
    // PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    // PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    // ProfiledPIDController thetaController = new ProfiledPIDController( // PID Controller for theta
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI); // keep the PIDController continuous

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     robotDrive::getP, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,
    //     xController,
    //     yController,
    //     thetaController,
    //     robotDrive::setModuleStates,
    //     robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }
  
  public enum autoC {
    Command_1,
    Command_2,
    Command_3,
    Command_4,
    Command_5,
    Command_6,
    Command_7,
    Command_8,
    Command_9
  }

  public void setUpShuffleBoard() {
    SendableChooser<autoC> autoCommand = new SendableChooser<autoC>();
      autoCommand.addOption("Command_1", autoC.Command_1);
      autoCommand.addOption("Command_2", autoC.Command_2);
      autoCommand.addOption("Command_3", autoC.Command_3);
      autoCommand.addOption("Command_4", autoC.Command_4);
      autoCommand.addOption("Command_5", autoC.Command_5);
      autoCommand.addOption("Command_6", autoC.Command_6);
      SmartDashboard.putData("Auto Type", autoCommand);
}

public void setFieldRelativeOffset(double offset) {
  robotDrive.setFieldRelativeOffset(offset);
}

  private boolean leftTrigger(){
    return (subController.getRawAxis(2) > 0.75);
  }

  public ProfiledPIDController getThetaController() {
    ProfiledPIDController thetaController = new ProfiledPIDController(1.25, 0, 0, new TrapezoidProfile.Constraints(2*Math.PI, 2*Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    return thetaController;
  }

  private Command Command_1(){
      Trajectory moveForwardTraj = TrajectoryGenerator.generateTrajectory(
          new Pose2d(new Translation2d(7.1927, 4.195), Rotation2d.fromDegrees(180)),
          List.of(),
          new Pose2d(new Translation2d(5.7133, 4.195), Rotation2d.fromDegrees(180)),
          AutoConstants.kTrajConfigSlow);
      SwerveControllerCommand moveToReefCommand = new SwerveControllerCommand(
          moveForwardTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return moveToReefCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command Command_2(){
    Trajectory moveToCoralStationTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(5.7133, 4.195), Rotation2d.fromDegrees(180)),
        new Pose2d(new Translation2d(5.7133, 4.795), Rotation2d.fromDegrees(200)),
        new Pose2d(new Translation2d(2.265, 6.205), Rotation2d.fromDegrees(300)),
        new Pose2d(new Translation2d(1.165, 7.092), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigStandard);
    SwerveControllerCommand moveToCoralStationCommand = new SwerveControllerCommand(
          moveToCoralStationTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return moveToCoralStationCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command command_3(){
    Trajectory rushTowardsReefTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(1.165, 7.092), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(2.025, 6.235), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(3.405, 5.375), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigFast);
    SwerveControllerCommand rushTowardsReefCommand = new SwerveControllerCommand(
          rushTowardsReefTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return rushTowardsReefCommand;
  }

  private Command command_4(){
    Trajectory moveToReefTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(3.405, 5.375), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(3.655, 4.948), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigSlow);
    SwerveControllerCommand moveToReefCommand = new SwerveControllerCommand(
          moveToReefTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return moveToReefCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command command_5(){
    Trajectory moveBackToCoralStationTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(3.655, 4.948), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(2.246, 6.386), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(1.165, 7.092), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigStandard);
    SwerveControllerCommand moveBackToCoralStationCommand = new SwerveControllerCommand(
          moveBackToCoralStationTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return moveBackToCoralStationCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command command_6(){
    Trajectory againRushTowardsReefTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(1.165, 7.092), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(2.195, 6.235), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(3.589, 5.375), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigFast);
    SwerveControllerCommand againRushTowardsReefCommand = new SwerveControllerCommand(
          againRushTowardsReefTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return againRushTowardsReefCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command command_7(){
    Trajectory againMoveToReefTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(3.589, 5.375), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(3.951, 5.097), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigSlow);
    SwerveControllerCommand againMoveToReefCommand = new SwerveControllerCommand(
          againMoveToReefTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return againMoveToReefCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command command_8(){
    Trajectory againMoveToCoralStationTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(3.951, 5.097), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(3.051, 6.266), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(1.165, 7.092), Rotation2d.fromDegrees(315))),
      AutoConstants.kTrajConfigStandard);
    SwerveControllerCommand againMoveToCoralStationCommand = new SwerveControllerCommand(
          againMoveToCoralStationTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return againMoveToCoralStationCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command command_9(){
    Trajectory againRushBackTowardsReefTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(1.165, 7.092), Rotation2d.fromDegrees(315)),
        new Pose2d(new Translation2d(2.170, 6.386), Rotation2d.fromDegrees(335)),
        new Pose2d(new Translation2d(2.93, 4.324), Rotation2d.fromDegrees(0))),
      AutoConstants.kTrajConfigFast);
    SwerveControllerCommand againRushBackTowardsReefCommand = new SwerveControllerCommand(
          againRushBackTowardsReefTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return againRushBackTowardsReefCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }

  private Command command_10(){
    Trajectory againMoveBackToReefTraj = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(new Translation2d(2.93, 4.324), Rotation2d.fromDegrees(0)),
        new Pose2d(new Translation2d(3.232, 4.182), Rotation2d.fromDegrees(0))),
      AutoConstants.kTrajConfigSlow);
    SwerveControllerCommand againMoveToReefCommand = new SwerveControllerCommand(
          againMoveBackToReefTraj,
          robotDrive::getP, 
          DriveConstants.kDriveKinematics, 
          new PIDController(1, 0, 0), 
          new PIDController(1, 0, 0),
          getThetaController(),
          robotDrive::setModuleStates,
          robotDrive);
      return againMoveToReefCommand.andThen(() -> robotDrive.drive(0, 0, 0, false, false));
  }
}
