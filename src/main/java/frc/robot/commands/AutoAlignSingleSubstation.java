// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.xml.xpath.XPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class AutoAlignSingleSubstation extends CommandBase {
  Arm s_Arm;
  SwerveDrive s_Swerve;
  PIDController xController = new PIDController(0.02, 0,0.0022);
  PIDController yController = new PIDController(Constants.Swerve.autoDriveCubePickupYP, Constants.Swerve.autoDriveCubePickupYI, Constants.Swerve.autoDriveCubePickupYD);
  PIDController rController = new PIDController(Constants.Swerve.autoDriveCubePickupRP, Constants.Swerve.autoDriveCubePickupRI, Constants.Swerve.autoDriveCubePickupRD);
  boolean isDone = false;
  int waitCnt = 0;
  double xSpeed = 0;
  double ySpeed = 0;
  double rSpeed = 0;
  ArmPosition currentTarget;
  int grabState = 0;
  double degrees = 0;
  Translation2d translation;
  /** Creates a new AutoGrabCube. */
  public AutoAlignSingleSubstation(SwerveDrive s_Swerve, Arm s_Arm) {
    this.s_Arm = s_Arm;
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve, s_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Arm.openClaw();
    rController.reset();
    xController.reset();
    yController.reset();
    s_Swerve.setHeadlights(false);
    isDone = false;
    waitCnt = 0;
    rSpeed = 0;
    xSpeed = 0;
    ySpeed = 0;
    s_Swerve.limelightDown();

    s_Swerve.setLimelightPipeline(3);//6
    s_Swerve.setLimelightLED(false);
    xController.setSetpoint(195);
    xController.setTolerance(12);
    yController.setSetpoint(Constants.Swerve.autoGrabCubeLidarTarget);
    yController.setTolerance(Constants.Swerve.autoGrabCubeYTolerance);
    if (DriverStation.getAlliance() == Alliance.Blue) {
      rController.setSetpoint(90);
    }
    else {
      rController.setSetpoint(-90);
    }
    rController.setTolerance(Constants.Swerve.autoGrabUprightConeRTolerance);
    if (s_Arm.getCurrentArmPosition() == Constants.ArmConstants.grabFromSingleStationPosition) {
      currentTarget = Constants.ArmConstants.grabFromSingleStationPosition;
    }
    else {
      currentTarget = Constants.ArmConstants.restToSingleSubstationIntermediatePosition;
    }
    grabState = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double currentArmError = s_Arm.MoveArm(currentTarget);
      
      switch(grabState) {
        case 0:
          if (currentArmError < 2) {
            currentTarget = Constants.ArmConstants.grabFromSingleStationPosition;
          }
          if (s_Swerve.limelightHasTarget() == 1) {
            degrees = s_Swerve.getYaw().getDegrees();
            // if (degrees < 0) {
            //   degrees += 360;
            // }
            var boundingBoxXY = s_Swerve.getBoundingBoxX();
            rSpeed = rController.calculate(degrees);
            xSpeed = xController.calculate(boundingBoxXY[1]); 
            if (xController.atSetpoint()) {
              //xSpeed = 0;
            }
            else {
              //whats here
            }
            if (boundingBoxXY[1] > 100 && boundingBoxXY[1] < 220) { //don't move forward until x is close enough
              ySpeed = 1.5; //2
            }
            else if (boundingBoxXY[1] <= 100) {
              xSpeed = 1.5;
            }
            else if (boundingBoxXY[1] >= 220) {
              xSpeed = -1.5;
            }
            else {
              ySpeed = 0;
            }
            if (rController.getPositionError() > 5) {
              xSpeed = xSpeed * 0.7;
              ySpeed = ySpeed * 0.7;
            }
            translation = new Translation2d(ySpeed, xSpeed);
            
            
            s_Swerve.drive(translation, rSpeed, false, true);
            // if (yController.atSetpoint() && xController.atSetpoint() && currentArmError < 3 && currentTarget == Constants.ArmConstants.grabCubePosition) {              
            //   s_Swerve.stop();
            // }            
        }
        else {
         // s_Swerve.stop();
        }
      break;
      case 1:
          s_Arm.closeClaw();
          waitCnt = 0;
          grabState++;
      break;
      case 2:
          waitCnt++;
          if (waitCnt > 10) {
            currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
            grabState++;
          }
      break;
      case 3:
          if (currentArmError < 3) {
            currentTarget = Constants.ArmConstants.restPosition;
            isDone = true;
          }
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.stop();
    s_Swerve.setHeadlights(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
