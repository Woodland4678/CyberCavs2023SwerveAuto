// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.LEDModes;
import edu.wpi.first.wpilibj.Timer;

public class AutoScoreHigh extends CommandBase {
  Arm s_Arm;
  SwerveDrive s_Swerve;
  PIDController xController = new PIDController(Constants.Swerve.autoDriveScoreXP, Constants.Swerve.autoDriveScoreXI, Constants.Swerve.autoDriveScoreXD);
  PIDController yController = new PIDController(Constants.Swerve.autoDriveScoreYP, Constants.Swerve.autoDriveScoreYI, Constants.Swerve.autoDriveScoreYD);
  PIDController rController = new PIDController(Constants.Swerve.autoDriveScoreRP, Constants.Swerve.autoDriveScoreRI, Constants.Swerve.autoDriveScoreRD);
  boolean isDone = false;
  int isInPosCnt = 0;
  int currentPipeline = Constants.Swerve.limelightHighScorePipeline;
  ArmPosition currentTarget;
  double yTarget = 0;
  boolean isHigh = true;
  double wristAdjustment = 0;
  boolean isMainArmPositionSet = false;
  boolean isAutoMode = false;
  ArmPosition originalTarget;
  CommandXboxController operatorJoystick;
  int waitScoreCnt = 0;
  double timeStart = 0;
  double xSpeed = 0;
  double rSpeed = 0;
  /** Creates a new ScoreHigh. */
  public AutoScoreHigh(Arm s_Arm, SwerveDrive s_Swerve, boolean isHigh, CommandXboxController operatorJoystick, boolean isAutoMode, int waitScoreCnt) {
    this.s_Arm = s_Arm;
    this.s_Swerve = s_Swerve;
    addRequirements(s_Arm, s_Swerve); 
    this.isHigh = isHigh;
    this.wristAdjustment = wristAdjustment;
    this.isAutoMode = isAutoMode;
    this.waitScoreCnt = waitScoreCnt;
    this.originalTarget = Constants.ArmConstants.scoreConeHighPosition;
    this.operatorJoystick = operatorJoystick;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xSpeed = 0;
    rSpeed = 0;
    timeStart = Timer.getFPGATimestamp();
    xController.reset();
    yController.reset();
    rController.reset();
    s_Swerve.setLimeLED(true);
    if (isHigh) {
      yTarget = Constants.Swerve.autoScoreHighYTarget;
      this.originalTarget = Constants.ArmConstants.scoreConeHighPosition;
    }
    else {
      yTarget = Constants.Swerve.autoScoreMediumYTarget;
      this.originalTarget = Constants.ArmConstants.scoreConeMediumPosition;
    }
    if (!isHigh) {
      currentTarget = Constants.ArmConstants.restToScoreMediumIntermediatePosition;
    }
    else {
      currentTarget = Constants.ArmConstants.restToScoreHighIntermediatePosition;
    }
    s_Swerve.limelightUp();
    //xController.setGoal(0);
    //yController.setGoal(0);
    xController.setSetpoint(1.75);
    if (!this.isAutoMode) {
      rController.setSetpoint(s_Swerve.getYaw().getDegrees());
    }
    else {
      if (s_Swerve.getYaw().getDegrees() < 0) {
        rController.setSetpoint(-180);
      }
      else {
        rController.setSetpoint(180);
      }
      
    }
    yController.setSetpoint(7.5); //8.2 
    xController.setTolerance(Constants.Swerve.autoDriveScoreXTolerance);
    yController.setTolerance(Constants.Swerve.autoDriveScoreYTolerance);
    rController.setTolerance(Constants.Swerve.autoDriveScoreRTolerance);
    currentPipeline = Constants.Swerve.limelightHighScorePipeline;
    s_Swerve.setLimelightPipeline(currentPipeline);
    isDone = false;
    isInPosCnt = 0;
    isMainArmPositionSet = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Swerve.getLimeLighttvert() < 18) {
      yController.setSetpoint(4.9);
    }
    else {
      yController.setSetpoint(7.5);
    }
    double currentTime = Timer.getFPGATimestamp();
    if (operatorJoystick.getRawAxis(1) > 0.1 && isMainArmPositionSet) {
      if (currentTarget.wristPitchTarget > -120) { 
        currentTarget.wristPitchTarget += -operatorJoystick.getRawAxis(1) * 2.5;
      }
    }
    else if (operatorJoystick.getRawAxis(1) < 0.1 && isMainArmPositionSet) {
      if (currentTarget.wristPitchTarget < 15) {
        currentTarget.wristPitchTarget += -operatorJoystick.getRawAxis(1) * 2.5;
      }
    }
    var currentArmError = s_Arm.MoveArm(currentTarget);
    SmartDashboard.putNumber("Auto score high arm error", currentArmError);
    if (currentArmError < 8 && !isMainArmPositionSet) {
      if (isHigh) {
        currentTarget = Constants.ArmConstants.scoreConeHighPosition;
        currentTarget.wristPitchTarget = -
        48;
      }
      else {
        currentTarget = Constants.ArmConstants.scoreConeMediumPosition;
        currentTarget.wristPitchTarget = -5;

      }
      isMainArmPositionSet = true;
    }
    
    if (s_Swerve.limelightHasTarget() == 1) {
      LimelightTarget_Retro currentLimelightTarget = s_Swerve.getBestLimelightTarget();
      if (currentLimelightTarget != null) {
        var degrees =s_Swerve.getYaw().getDegrees();
        if (rController.getSetpoint() > 0 && degrees < 0) {
          degrees = 360 + degrees;
        }
        else if (rController.getSetpoint() < 0 && degrees > 0) {
          degrees = degrees - 360;
        }
        
        var ySpeed = yController.calculate(s_Swerve.getLimelightY());
        //if (yController.atSetpoint()) {
        //  ySpeed = 0;
       // }
        // if (Math.abs(xController.getPositionError()) > 7.5) {
        //   ySpeed = 0;
        // }
        //if (!rController.atSetpoint()) {
          rSpeed = rController.calculate(degrees);
        //}
        //else {
        //  rSpeed = 0;
       // }
        //if (!xController.atSetpoint()) {
          xSpeed = xController.calculate(s_Swerve.getLimelightX());
        //}
        //else {
         // xSpeed = 0;
        //}
        
        // if (Math.abs(rSpeed) < 0.7) {
        //   xSpeed = rLimiter.calculate(rController.calculate(s_Swerve.getYaw().getDegrees()));
        // }
        // else {
        //   ySpeed = ySpeed * 0.5;
        //   xSpeed = 0;
        // }
        Translation2d translation = new Translation2d(ySpeed, xSpeed);
        SmartDashboard.putNumber(
                    "Auto Score high arm error",currentArmError);        
        SmartDashboard.putBoolean(
                      "Auto score y controller",yController.atSetpoint());
        SmartDashboard.putBoolean(
                      "Auto score x controller",xController.atSetpoint());
        SmartDashboard.putBoolean(
                      "Auto score r controller",rController.atSetpoint());
        SmartDashboard.putNumber(
                        "Auto score x Speed",xSpeed);
        SmartDashboard.putNumber(
                      "Auto score y Speed",ySpeed);
        SmartDashboard.putNumber(
                      "Auto score r Speed",rSpeed);
        
        
        if (xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint() && currentArmError < 10) {
          isInPosCnt++;
          s_Arm.setLEDMode(LEDModes.SOLIDGREEN);
        }
        else {
          isInPosCnt = 0;
          s_Arm.setLEDMode(LEDModes.SOLIDRED);
        }
        if (isInPosCnt > waitScoreCnt || (currentTime - timeStart) > 1.45) { //1.75 second time out
          if (isInPosCnt > waitScoreCnt) {
            s_Arm.setLEDMode(LEDModes.SOLIDGREEN);
            s_Swerve.stop();
          }
          else {
            s_Arm.setLEDMode(LEDModes.SOLIDRED);
          }
          if (DriverStation.isAutonomous()) {
            isDone = true;
          }          
          s_Swerve.stop();
        }
        else {
          
          s_Swerve.drive(translation, rSpeed, false, true);
          
        }
      }
    }
    else {
      s_Swerve.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.limelightDown();
    s_Swerve.stop();
    s_Swerve.setLimeLED(false);
    if (!DriverStation.isAutonomous()) {
      if (s_Arm.getGamePieceMode() == Constants.ArmConstants.coneMode) {
        s_Arm.setLEDMode(LEDModes.SOLIDYELLOW);
      }
      else {
        s_Arm.setLEDMode(LEDModes.SOLIDPURPLE);
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
