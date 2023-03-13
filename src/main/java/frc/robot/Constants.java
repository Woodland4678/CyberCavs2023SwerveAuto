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
    public static final int limelightSolenoidChannel1 = 6;
    public static final int limelightSolenoidChannel2 = 7;

    public static final int gyroId = 6;
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double trackWidth = Units.inchesToMeters(22.5); //0.6223 practice bot //Units.inchesToMeters(21.73);
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double wheelBase = Units.inchesToMeters(26.75); //0.62865 practice bot
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
    public static final int driveContinuousCurrentLimit = 40;

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
    public static final double maxAngularVelocity = 6; //1.5

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
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees( 49.5); 
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 0;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(212.5);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(122.23); //81.7
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 16;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(81.01); //122.2
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }


    public static int driveAssistCANId = 15;
    /////////////////////////////////////////////////////////////
    //**Auto drive values (for scoring and game piece pickup) */
    /////////////////////////////////////////////////////////////
    public static double autoDriveScoreXP = 0.1;
    public static double autoDriveScoreXI = 0.0;
    public static double autoDriveScoreXD = 0.0;
    public static double autoDriveScoreXTolerance = 0.75;
    public static double autoDriveScoreYP = 0.1;
    public static double autoDriveScoreYI = 0.0;
    public static double autoDriveScoreYD = 0.0;
    public static double autoDriveScoreYTolerance = 0.75;
    public static double autoDriveScoreRP = 0.1;
    public static double autoDriveScoreRI = 0.0;
    public static double autoDriveScoreRD = 0.0;
    public static double autoDriveScoreRTolerance = 1;

    public static double autoScoreHighYTarget = 1;
    public static double autoScoreMediumYTarget = -5;

    public static double autoDriveConePickupXP = 0.01;
    public static double autoDriveConePickupXI = 0.0;
    public static double autoDriveConePickupXD = 0.0;
    public static double autoDriveConePickupXTolerance = 1;
    public static double autoDriveConePickupYP = 0.5;
    public static double autoDriveConePickupYI = 0.0;
    public static double autoDriveConePickupYD = 0.0;
    public static double autoDriveConePickupYTolerance = 1;
    public static double autoDriveConePickupRP = 0.15;
    public static double autoDriveConePickupRI = 0.0;
    public static double autoDriveConePickupRD = 0.0;
    public static double autoDriveConePickupRTolerance = 1;

    public static double autoDriveCubePickupXP = 0.05;
    public static double autoDriveCubePickupXI = 0.0;
    public static double autoDriveCubePickupXD = 0.0;
    public static double autoDriveCubePickupXTolerance = 2;
    public static double autoDriveCubePickupYP = 0.022;
    public static double autoDriveCubePickupYI = 0.0;
    public static double autoDriveCubePickupYD = 0.0;
    public static double autoDriveCubePickupYTolerance = 5;
    public static double autoDriveCubePickupRP = 0.15;
    public static double autoDriveCubePickupRI = 0.0;
    public static double autoDriveCubePickupRD = 0.0;
    public static double autoDriveCubePickupRTolerance = 1;
    
    public static double autoGrabCubeLidarTarget = 25;
    public static double autoGrabCubeYTolerance = 3;
    public static double autoGrabCubeEnableY = 4.0;
    

    public static double coneAutoDriveYTarget = 131;
    public static double cubeAutoDriveYTarget = 131;

    public static int limelightHighScorePipeline = 3;
    public static int limelightMediumScorePipeline = 5;
    
    public static int distanceLaserLeftChannel = 6;
    public static int distanceLaserCenterChannel = 7;
   // public static int distanceLaserRightChannel = 8;
    

    public static final double uprightConeAutoDriveYTarget = 33.6;

    public static final int headlightsRelayChannel = 0;

    /*Constants for auto grab upright cone with the lidar */
    public static final double autoGrabUprightConeYSwitchToLidar = 42;
    public static final double autoGrabUprightConeRTolerance = 1;
    public static final double autoGrabUprightConeYLimelightTolerance = 1;
    public static final double autoGrabUprightConeYLimelightTarget = 40.0;
    public static final double autoGrabUprightConeXLimelightTolerance = 7;
    public static final double autoGrabUprightConeLidarYP = 0.022;
    public static final double autoGrabUprightConeLidarYI = 0.0;
    public static final double autoGrabUprightConeLidarYD = 0.0;
    public static final double autoGrabUprightConeLidarYTarget = 49; //cm distance we want to be to the cone
    public static final double autoGrabUprightConeLidarYTolerance = 5;
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
    public static final int shoulderLeaderMotorCanId = 10;
    public static final int shoulderFollowerMotorCanId = 9;

    public static final int elbowLeaderMotorCanId = 11;
    public static final int elbowFollowerMotorCanId = 12;

    public static final int wristPitchCanId = 13;
    public static final int wristRollCanId = 14;

    public static final int pneumaticClawChannel = 1;
    public static final int pneumaticClawChannel2 = 2;

    public static final double shoulderGearRatio = (60.0 / 1.0); 
    public static final double elbowGearRatio = ((((64/12) * 70) / 28) * 5); 
    public static final double wristRollGearRatio = (90/12); 
    public static final double wristPitchGearRatio = (75.0/ 1.0);

    public static final double elbowAngleOffset = 105.35;
    public static final double shoulderAngleOffset = 130.25;

    public static final double shoulderAngleConversionFactor = (360 / shoulderGearRatio);
    public static final double elbowAngleConversionFactor = 360 / elbowGearRatio;
    public static final double wristRollAngleConversionFactor = 360 / wristRollGearRatio;
    public static final double wristPitchAngleConversionFactor = 360 / wristPitchGearRatio;

    public static final double shoulderLength = 37.0;
    public static final double elbowLength = 36.0;

    public static final int shoulderEncoderAbsoluteID = 5;
    public static final int elbowEncoderAbsoluteID = 4;

    public static final double shoulderP = 0.015;
    public static final double shoulderI = 0.0;
    public static final double shoulderD = 1;
    public static final double shoulderFF = 0.00;

    public static final double elbowP = 0.015;
    public static final double elbowI = 0.0;
    public static final double elbowD = 1;
    public static final double elbowFF = 0.00;

    public static final double wristPitchP = 0.01;
    public static final double wristPitchI = 0.0;
    public static final double wristPitchD = 0.0;
    public static final double wristPitchFF = 0.0;

    public static final double wristRollP = 0.013;
    public static final double wristRollI = 0.0;
    public static final double wristRollD = 0.0;
    public static final double wristRollFF = 0.0;

    public static final double position1X = 52; //inches
    public static final double position1Y = 40;
    public static final double position1WristAngle = -30;
    public static final double position1WristRollAngle = 35;

    public static final double position2X = 15;
    public static final double position2Y = 15;
    public static final double position2WristAngle = 30;
    public static final double position2WristRollAngle = -35;

    public static final int cubeMode = 0;
    public static final int coneMode = 1;


    //******These define the area we want the arm to avoid (in this case where the shelf is that protects the limelight) */
    public static final double armExclusionXMin = 7.0;
    public static final double armExclusionXMax = 13.0;
    public static final double armExclusionY = 4.0;

    public static final int wristPitchLimitSwitchChannel = 9;

    
    public static final ArmPosition scoreConeHighPosition = new ArmPosition(47.5, 44.5, 0, -35, false);
    public static ArmPosition pickupPosition = new ArmPosition(28.84, 11.5, 0, -127.6, false);//wrist roll position needs to change so this shouldn't be "final"
    public static final ArmPosition pos2 = new ArmPosition(20, 20, 0, 0, false);
    public static final ArmPosition restPosition = new ArmPosition(110.3, -170.46, 0, 0, true);
    public static final ArmPosition scoreConeMediumPosition = new ArmPosition(35.8, 28, 0, 5, false);
    public static final ArmPosition scoreLowPosition = new ArmPosition(25, -5, 0, -20, false);
    public static ArmPosition grabConePosition = new ArmPosition(31.27, 2.3, 0, -140.68, false);
    public static ArmPosition grabCubePosition = new ArmPosition(33.1, 8, 0, -127.6, false);
    public static final ArmPosition pickupToRestIntermediatePosition = new ArmPosition(17.44, 10.2, 0, 0, false);
    public static final ArmPosition yeetCubePosition = new ArmPosition(22.8, 47.8, 0, 0, false);
    public static final ArmPosition grabUprightConePosition = new ArmPosition(32.7, -3.5, 0, 2, false);
    public static final ArmPosition restToScoreHighIntermediatePosition = new ArmPosition(18.205, 36, 0, 0, false);
    public static final ArmPosition restToScoreMediumIntermediatePosition = new ArmPosition(21.4, 24.2, 0, -38.51, false);

    public static final ArmPosition scoreCubeHighPosition = new ArmPosition(48, 45, 0, -100, false);
    public static final ArmPosition scoreCubeMediumPosition = new ArmPosition(40, 35, 0, -100, false);
    public static final ArmPosition pickupUprightIntermediatePosition = new ArmPosition(15.44, 10.2, 0, 0, false);

  }
  public static enum LEDModes {
    OFF,
    SOLIDYELLOW,
    SOLIDPURPLE,
    BLINKYELLOW,
    BLINKPURPLE,
    SOLIDGREEN,
    BLINKGREEN,
    RAINBOW,
    SOLIDRED,
    SOLIDBLUE
  }
  public static class ArmPosition{
    public double xTarget = 10; //default "homeish" position
    public double yTarget = 15; //default "homeish" position
    public double wristPitchTarget = 0;
    public double wristRollTarget = 0;
    public boolean isAngleTarget = false;
    public ArmPosition(double xTarget, double yTarget, double wristRollTarget, double wristPitchTarget, boolean isAngleTarget) {
      this.xTarget = xTarget;
      this.yTarget = yTarget;
      this.wristPitchTarget = wristPitchTarget;
      this.wristRollTarget = wristRollTarget;
      this.isAngleTarget = isAngleTarget;
    }
  }
}
