// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.ArmPosition;

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
  /** Creates a new ScoreHigh. */
  public AutoScoreHigh(Arm s_Arm, SwerveDrive s_Swerve, boolean isHigh) {
    this.s_Arm = s_Arm;
    this.s_Swerve = s_Swerve;
    addRequirements(s_Arm, s_Swerve);
    this.isHigh = isHigh;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    rController.reset();
    s_Swerve.setLimeLED(true);
    if (isHigh) {
      yTarget = Constants.Swerve.autoScoreHighYTarget;
    }
    else {
      yTarget = Constants.Swerve.autoScoreMediumYTarget;
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
    xController.setSetpoint(0);
    rController.setSetpoint(s_Swerve.getYaw().getDegrees());
    yController.setSetpoint(4.4);
    xController.setTolerance(Constants.Swerve.autoDriveScoreXTolerance);
    yController.setTolerance(Constants.Swerve.autoDriveScoreYTolerance);
    rController.setTolerance(Constants.Swerve.autoDriveScoreRTolerance);
    currentPipeline = Constants.Swerve.limelightHighScorePipeline;
    s_Swerve.setLimelightPipeline(currentPipeline);
    isDone = false;
    isInPosCnt = 0;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    var currentArmError = s_Arm.MoveArm(currentTarget);
    if (currentArmError < 8) {
      if (isHigh) {
        currentTarget = Constants.ArmConstants.scoreConeHighPosition;
      }
      else {
        currentTarget = Constants.ArmConstants.scoreConeMediumPosition;
      }
    }
    
    if (s_Swerve.limelightHasTarget() == 1) {
      LimelightTarget_Retro currentLimelightTarget = s_Swerve.getBestLimelightTarget();
      if (currentLimelightTarget != null) {
        var degrees =s_Swerve.getYaw().getDegrees();
        if (degrees < 0) {
          degrees = 360 + degrees;
        }
        var rSpeed = rController.calculate(degrees);
        var xSpeed = xController.calculate(s_Swerve.getLimelightX());
        var ySpeed = yController.calculate(s_Swerve.getLimelightY());
        // if (Math.abs(rSpeed) < 0.7) {
        //   xSpeed = rLimiter.calculate(rController.calculate(s_Swerve.getYaw().getDegrees()));
        // }
        // else {
        //   ySpeed = ySpeed * 0.5;
        //   xSpeed = 0;
        // }
        Translation2d translation = new Translation2d(ySpeed, xSpeed);
        // SmartDashboard.putNumber(
        //             "xSpeed",xSpeed);
        // SmartDashboard.putNumber(
        //               "ySpeed",ySpeed);
        // SmartDashboard.putNumber(
        //               "rSpeed",rSpeed);
        
        
        if (xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint()) {
          isInPosCnt++;
        }
        else {
          isInPosCnt = 0;
        }
        if (isInPosCnt > 10) {
          isDone = true;
          s_Arm.setLEDs(0, 255, 0);
          s_Swerve.stop();
        }
        else {
          s_Swerve.drive(translation, 0, false, true);
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
    //s_Swerve.limelightDown();
    s_Swerve.stop();
    s_Swerve.setLimeLED(false);
    if (s_Arm.getGamePieceMode() == Constants.ArmConstants.coneMode) {
      s_Arm.coneMode();
    }
    else {
      s_Arm.cubeMode();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
