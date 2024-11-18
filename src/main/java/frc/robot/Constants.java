// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class DriveConstants {
        public static final int kFrontLeftDriveCANID = 1;
        public static final int kFrontRightDriveCANID = 3;
        public static final int kBackLeftDriveCANID = 7;
        public static final int kBackRightDriveCANID = 5;

        public static final int kFrontLeftSteerCANID = 2;
        public static final int kFrontRightSteerCANID = 4;
        public static final int kBackLeftSteerCANID = 8;
        public static final int kBackRightSteerCANID = 6;

        public static final double rotationSlewRate = 2.0;
        public static final double directionSlewRate = 1.2;
        public static final double magLimiterSlewRate = 1.8;

        public static final double kTrackWidth = Units.inchesToMeters(25);
        public static final double kTrackLength = Units.inchesToMeters(25);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kTrackLength / 2, kTrackWidth / 2),
                new Translation2d(kTrackLength / 2, -kTrackWidth / 2),
                new Translation2d(-kTrackLength / 2, kTrackWidth / 2),
                new Translation2d(-kTrackLength / 2, -kTrackWidth / 2));

        public static final double kFrontLeftOffset = (-Math.PI / 2);
        public static final double kFrontRightOffset = 0;
        public static final double kBackLeftOffset = Math.PI;
        public static final double kBackRightOffset = (Math.PI / 2);

        public static final double kMaxSpeedMetersPerSec = 4.0; // max speed in mps
        public static final double kMaxAngSpeedRadiansPerSec = 2 * Math.PI; // max turning speed in rps
    }

    public static class ModuleConstants {
        public static final int kDrivingMotorPinionTeeth = 13;

        public static final boolean kTurningEncoderInverted = true;

        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kCoPilotControllerPort = 1;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    public static final class LEDConstants {
        public static final int kLEDBarPWM = 9;
        public static final int ledLength = 40;
        public static final int ledBufferLength = 40;
    }

    public static final class TestMotorConstants {
        public static final int kTestRotMotorCanId = 9;
        public static final int kTestPosMotorCanId = 10;

        public static final double kTestMotorP = 0.0005;
        public static final double kTestMotorI = 0;
        public static final double kTestMotorD = 0;
        public static final double kTestMotorFF = 0.00007;

        public static final double kTestPosMotorP = 0.0005;
        public static final double kTestPosMotorI = 0;
        public static final double kTestPosMotorD = 0;
        public static final double kTestPosMotorFF = 0.00007;

        public static final double kTestMotorSpeedSlow = 800;
        public static final double kTestMotorSpeedMedium = 1200;
        public static final double kTestMotorSpeedNormal = 2000;

        public static final double kTestMotorTopPosition = 100;
        public static final double kTestMotorIdlePosition = 0;
        public static final double kTestPosMotorSpeed = 1;

        public static final double kTestMotorMinOutput = -1;
        public static final double kTestMotorMaxOutput = 1;
        public static final double kTestPosMotorMinOutput = -0.4;
        public static final double kTestPosMotorMaxOutput = 0.7;


        public static final double kTestMotorSpeedDeadband = 250; // sets a safe range for the motor to be stay out of corrections
        public static final double kTestPosMotorPositionDeadband = 3;
        
    }
}
