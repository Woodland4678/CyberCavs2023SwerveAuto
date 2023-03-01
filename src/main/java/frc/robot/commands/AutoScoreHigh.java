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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class AutoScoreHigh extends CommandBase {
  Arm s_Arm;
  SwerveDrive s_Swerve;
  PIDController xController = new PIDController(0.2, 0.01, 0);
  PIDController yController = new PIDController(0.2, 0.01, 0);
  PIDController rController = new PIDController(0.15, 0.0005, 0);
  boolean isDone = false;
  int isInPosCnt = 0;
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
    //xController.setGoal(0);
    //yController.setGoal(0);
    xController.setSetpoint(0);
    rController.setSetpoint(180);
    yController.setSetpoint(20.48);
    xController.setTolerance(1);
    yController.setTolerance(0.5);
    s_Swerve.setLimelightPipeline(3);
    isDone = false;
    isInPosCnt = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Arm.MoveArm(Constants.ArmConstants.scoreConeHighPosition);
    var robotPose = new Pose2d();
    if (s_Swerve.limelightHasTarget() == 1) {
      var degrees =s_Swerve.getYaw().getDegrees();
      if (degrees < 0) {
        degrees += 360;
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
      
      s_Swerve.drive(translation, rSpeed, false, true);
      if (Math.abs(s_Swerve.getLimelightY() - 20.48) < 0.5) {
        isInPosCnt++;
      }
      else {
        isInPosCnt = 0;
      }
      if (isInPosCnt > 20) {
        isDone = true;
      }
    }
    else {
      s_Swerve.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
