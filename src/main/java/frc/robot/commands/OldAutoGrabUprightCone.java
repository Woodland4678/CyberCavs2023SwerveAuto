// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class OldAutoGrabUprightCone extends CommandBase {
  Arm s_Arm;
  SwerveDrive s_Swerve;
  PIDController xController = new PIDController(Constants.Swerve.autoDriveConePickupXP, Constants.Swerve.autoDriveConePickupXI, Constants.Swerve.autoDriveConePickupXD);
  PIDController yController = new PIDController(Constants.Swerve.autoDriveConePickupYP, Constants.Swerve.autoDriveConePickupYI, Constants.Swerve.autoDriveConePickupYD);
  PIDController rController = new PIDController(0.07, 0.0001, 0.005);
  boolean isDone = false;
  int isInPosCnt = 0;
  int isDoneCnt = 0;
  double limelightYTarget = 0;
  ArmPosition currentTarget;
  double currentArmError = 0;
  int grabState = 0;
  PathPlannerTrajectory moveToCone;
  /** Creates a new AutoGrabUprightCone. */
  public OldAutoGrabUprightCone(Arm s_Arm, SwerveDrive s_Swerve) {
    this.s_Arm = s_Arm;
    this.s_Swerve = s_Swerve;
    addRequirements(s_Arm, s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    grabState = 0;
    xController.reset();
    yController.reset();
    rController.reset();
    currentArmError = 50.0;
    isInPosCnt = 0;
    currentTarget = Constants.ArmConstants.pickupUprightIntermediatePosition;
    s_Swerve.limelightDown();
    isDoneCnt = 0;
    if (s_Arm.getGamePieceMode() == Constants.ArmConstants.coneMode) {
      limelightYTarget = Constants.Swerve.uprightConeAutoDriveYTarget;
    }
    else {
      limelightYTarget = Constants.Swerve.cubeAutoDriveYTarget;
    }
    xController.setSetpoint(0);
    rController.setSetpoint(s_Swerve.getYaw().getDegrees());
    yController.setSetpoint(limelightYTarget);
    xController.setTolerance(1);
    yController.setTolerance(1);
    if (s_Arm.gamePieceMode == Constants.ArmConstants.coneMode) {
      s_Swerve.setLimelightPipeline(1);
    }
    else {
      s_Swerve.setLimelightPipeline(2);
    }
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var boundingBox = s_Swerve.getBoundingBoxX();
    switch(grabState) {
      case 0:
        //currentArmError = s_Arm.MoveArm(currentTarget);
        if (boundingBox[0] > 40) {          
          grabState++;
          currentTarget = Constants.ArmConstants.grabUprightConePosition;
          double distanceFromFrameToCone = 121.698104 - 2.55144332 * boundingBox[0] + 0.0154124386 * boundingBox[0] * boundingBox[0];
          SmartDashboard.putNumber(
                          "Distance From Cone",distanceFromFrameToCone);
          moveToCone = PathPlanner.generatePath(
              new PathConstraints(2, 2), 
              new PathPoint(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), s_Swerve.getCurrentVelocity()), // position, heading(direction of travel), holonomic rotation, velocity override
              new PathPoint(new Translation2d(0.1, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
          );
        }
        else  {
          if (s_Swerve.limelightHasTarget() == 1) {
            var degrees =s_Swerve.getYaw().getDegrees();
            if (degrees < 0) {
              degrees += 360;
            }
            //var boundingBoxXY = s_Swerve.getBoundingBoxX();
            var rSpeed = rController.calculate(degrees);
            var xSpeed = xController.calculate(s_Swerve.getLimelightX()); //bounding box [1]
            var ySpeed = yController.calculate(s_Swerve.getLimelightY()); // bounding box [2]
            // if (Math.abs(rSpeed) < 0.7) {
            //   xSpeed = rLimiter.calculate(rController.calculate(s_Swerve.getYaw().getDegrees()));
            // }
            // else {
            //   ySpeed = ySpeed * 0.5;
            //   xSpeed = 0;
            // }
            Translation2d translation = new Translation2d(-ySpeed, xSpeed);
            SmartDashboard.putNumber(
                        "xSpeed",xSpeed);
            SmartDashboard.putNumber(
                          "ySpeed",ySpeed);
            SmartDashboard.putNumber(
                          "rSpeed",rSpeed);
            
            s_Swerve.drive(translation, rSpeed, false, true);
          }
        }
      break;
      case 1:
        //s_Swerve.stop();
        isDone = true;
        // grabState++;
        new RunCommand(() -> s_Swerve.followTrajectoryCommand(moveToCone, true),
          s_Swerve);
          
      break;
      case 2:
        //s_Swerve.stop();
      break;
    }
    /*
    if (xController.atSetpoint() && yController.atSetpoint() && currentArmError < 3){
      s_Swerve.stop();
      currentTarget = Constants.ArmConstants.grabUprightConePosition;
    }
    else if (!(currentTarget == Constants.ArmConstants.grabUprightConePosition)) {
      if (s_Swerve.limelightHasTarget() == 1) {
        var degrees =s_Swerve.getYaw().getDegrees();
        if (degrees < 0) {
          degrees += 360;
        }
        //var boundingBoxXY = s_Swerve.getBoundingBoxX();
        var rSpeed = rController.calculate(degrees);
        var xSpeed = xController.calculate(s_Swerve.getLimelightX()); //bounding box [1]
        var ySpeed = yController.calculate(s_Swerve.getLimelightY()); // bounding box [2]
        // if (Math.abs(rSpeed) < 0.7) {
        //   xSpeed = rLimiter.calculate(rController.calculate(s_Swerve.getYaw().getDegrees()));
        // }
        // else {
        //   ySpeed = ySpeed * 0.5;
        //   xSpeed = 0;
        // }
        Translation2d translation = new Translation2d(-ySpeed, xSpeed);
        SmartDashboard.putNumber(
                    "xSpeed",xSpeed);
        SmartDashboard.putNumber(
                      "ySpeed",ySpeed);
        SmartDashboard.putNumber(
                      "rSpeed",rSpeed);
        
        s_Swerve.drive(translation, rSpeed, false, true);
      }
    }
    if (currentTarget == Constants.ArmConstants.grabUprightConePosition) {
      if (currentArmError < 2) {
        isInPosCnt++;
       
      }
      if (isInPosCnt > 20) {
        s_Arm.closeClaw();
      }
    }*/
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //s_Swerve.stop();
    //s_Arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
