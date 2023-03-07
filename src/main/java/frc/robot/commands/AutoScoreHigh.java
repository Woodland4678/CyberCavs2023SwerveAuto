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
  PIDController xController = new PIDController(Constants.Swerve.autoDriveXP, Constants.Swerve.autoDriveXI, Constants.Swerve.autoDriveXD);
  PIDController yController = new PIDController(Constants.Swerve.autoDriveScoreYP, Constants.Swerve.autoDriveScoreYI, Constants.Swerve.autoDriveScoreYD);
  PIDController rController = new PIDController(Constants.Swerve.autoDriveRP, Constants.Swerve.autoDriveRI, Constants.Swerve.autoDriveRD);
  boolean isDone = false;
  int isInPosCnt = 0;
  int currentPipeline = Constants.Swerve.limelightHighScorePipeline;
  ArmPosition currentTarget;
  double yTarget = Constants.Swerve.autoScoreHighYTarget;
  /** Creates a new ScoreHigh. */
  public AutoScoreHigh(Arm s_Arm, SwerveDrive s_Swerve) {
    this.s_Arm = s_Arm;
    this.s_Swerve = s_Swerve;
    addRequirements(s_Arm, s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentTarget = Constants.ArmConstants.restToScoreIntermediatePosition;
    s_Swerve.limelightUp();
    //xController.setGoal(0);
    //yController.setGoal(0);
    xController.setSetpoint(0);
    rController.setSetpoint(180);
    yController.setSetpoint(yTarget);
    xController.setTolerance(1);
    yController.setTolerance(0.5);
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
      currentTarget = Constants.ArmConstants.scoreConeHighPosition;
    }
    
    if (s_Swerve.limelightHasTarget() == 1) {
      LimelightTarget_Retro currentLimelightTarget = s_Swerve.getBestLimelightTarget(currentPipeline);
      if (currentLimelightTarget != null) {
        var degrees =s_Swerve.getYaw().getDegrees();
        if (degrees < 0) {
          degrees += 360;
        }
        var rSpeed = rController.calculate(degrees);
        var xSpeed = xController.calculate(currentLimelightTarget.tx);
        var ySpeed = yController.calculate(currentLimelightTarget.ty);
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
        
        
        if (Math.abs(s_Swerve.getLimelightY() - yTarget) < 0.5) {
          isInPosCnt++;
        }
        else {
          isInPosCnt = 0;
        }
        if (isInPosCnt > 10) {
          isDone = true;
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
    //s_Swerve.limelightDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
