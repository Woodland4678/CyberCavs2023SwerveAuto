// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.config.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int gyroId = 6;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double trackWidth = 0.6223; //Units.inchesToMeters(21.73);
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double wheelBase = 0.62865;
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (21.42857 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        //(wheelDiameter * Math.PI) / (driveGearRatio * 42); //42 is the number of "clicks" per full rotation of a neo motor
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees( 211.4);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(350);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(319.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(303);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5; //1.75
    public static final double kPYController = 3; //1.75
    public static final double kPThetaController = 1; //1.3

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
  public static final class IntakeConstants{
    public static final int leftMotorCanId = 69; //TODO if we add intake change these
    public static final int rightMotorCanId = 420;
  }

  public static final class ArmConstants{
    public static final int shoulderLeaderMotorCanId = 9;
    public static final int shoulderFollowerMotorCanId = 10;

    public static final int elbowLeaderMotorCanId = 11;
    public static final int elbowFollowerMotorCanId = 12;

    public static final int wristVerticalCanId = 13;
    public static final int wristHorizontalCanId = 14;

    public static final int pneumaticClawChannel = 1;

    public static final double shoulderGearRatio = (60 / 1.0); // TODO what is the actual ratio?
    public static final double elbowGearRatio = (60 / 1.0); // TODO what is the actual rati0
    public static final double wristHorizontalGearRatio = (60 / 1.0); // TODO what is the actual ratio
    public static final double wristVerticalGearRatio = (60 / 1.0); // TODO what is the actual ratio

    public static final double elbowAngleOffset = 0;
    public static final double shoulderAngleOffset = 0;

    public static final double shoulderAngleConversionFactor = 360 / shoulderGearRatio;
    public static final double elbowAngleConversionFactor = 360 / elbowGearRatio;
    public static final double wristHorizontalAngleConversionFactor = 360 / wristHorizontalGearRatio;
    public static final double wristVerticalAngleConversionFactor = 360 / wristVerticalGearRatio;

    public static final double shoulderLength = 37;
    public static final double elbowLength = 36;

    public static final int shoulderEncoderAbsoluteID = 4;
    public static final int elbowEncoderAbsoluteID = 5;

    public static final double shoulderP = 0.1;
    public static final double shoulderI = 0.0;
    public static final double shoulderD = 0.0;
    public static final double shoulderFF = 0.0;

    public static final double elbowP = 0.1;
    public static final double elbowI = 0.0;
    public static final double elbowD = 0.0;
    public static final double elbowFF = 0.0;

    public static final double wristVerticalP = 0.1;
    public static final double wristVerticalI = 0.0;
    public static final double wristVerticalD = 0.0;
    public static final double wristVerticalFF = 0.0;

    public static final double wristHorizontalP = 0.1;
    public static final double wristHorizontalI = 0.0;
    public static final double wristHorizontalD = 0.0;
    public static final double wristHorizontalFF = 0.0;
  }
}
