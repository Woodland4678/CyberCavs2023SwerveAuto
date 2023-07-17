// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class AutoGrabCube extends CommandBase {
  Arm s_Arm;
  SwerveDrive s_Swerve;
  PIDController xController = new PIDController(Constants.Swerve.autoDriveCubePickupXP, Constants.Swerve.autoDriveCubePickupXI, Constants.Swerve.autoDriveCubePickupXD);
  PIDController yController = new PIDController(Constants.Swerve.autoDriveCubePickupYP, Constants.Swerve.autoDriveCubePickupYI, Constants.Swerve.autoDriveCubePickupYD);
  PIDController rController = new PIDController(Constants.Swerve.autoDriveCubePickupRP, Constants.Swerve.autoDriveCubePickupRI, Constants.Swerve.autoDriveCubePickupRD);
  boolean isDone = false;
  int waitCnt = 0;
  double xSpeed = 0;
  double ySpeed = 0;
  double rSpeed = 0;
  int isInPositionCnt = 0;
  ArmPosition currentTarget;
  int grabState = 0;
  double degrees = 0;
  Translation2d translation;
  double maxLidarValue = 50;
  /** Creates a new AutoGrabCube. */
  public AutoGrabCube(SwerveDrive s_Swerve, Arm s_Arm) {
    this.s_Arm = s_Arm;
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve, s_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yController.setPID(Constants.Swerve.autoDriveCubePickupYP, Constants.Swerve.autoDriveCubePickupYI, Constants.Swerve.autoDriveCubePickupYD);
    maxLidarValue = 50;
    s_Arm.openClaw();
    rController.reset();
    xController.reset();
    yController.reset();
    s_Swerve.setHeadlights(true);
    isDone = false;
    waitCnt = 0;
    rSpeed = 0;
    xSpeed = 0;
    ySpeed = 0;
    s_Swerve.limelightDown();
    s_Swerve.setLimelightPipeline(0); //was 0
    s_Swerve.setLimelightLED(false);
    xController.setSetpoint(0);
    xController.setTolerance(Constants.Swerve.autoDriveCubePickupXTolerance);
    yController.setSetpoint(Constants.Swerve.autoGrabCubeLidarTarget);
    yController.setTolerance(Constants.Swerve.autoGrabCubeYTolerance);
    rController.setSetpoint(s_Swerve.getYaw().getDegrees());
    rController.setTolerance(Constants.Swerve.autoGrabUprightConeRTolerance);
    currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
    grabState = 0;
    isInPositionCnt = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var testPIDs = s_Swerve.getRotationPID();
    rController.setPID(testPIDs[0], testPIDs[1], testPIDs[2]);
      double currentArmError = s_Arm.MoveArm(currentTarget);
      
      switch(grabState) {
        case 0:
          if (currentArmError < 4) {
            if (currentTarget != Constants.ArmConstants.grabCubePosition) {
              currentTarget = Constants.ArmConstants.grabCubePosition;
              currentArmError = 20;
            }
          }
          if (s_Swerve.limelightHasTarget() == 1) {
            degrees = s_Swerve.getYaw().getDegrees();
            // if (degrees < 0) {
            //   degrees += 360;
            // }
            //var boundingBoxXY = s_Swerve.getBoundingBoxX();
            rSpeed = rController.calculate(degrees);
            xSpeed = xController.calculate(s_Swerve.getLimelightX());
            double yMeasurement = s_Swerve.getLimelightY();
            
            // if (s_Swerve.getlimelightLowestYValue() > 195) {
            //   yMeasurement = yController.getSetpoint() - 3;
            // }
            ySpeed = yController.calculate(yMeasurement);
            // if (Math.abs(s_Swerve.getLimelightX()) < Constants.Swerve.autoGrabCubeEnableY) { //don't move forward until x is close enough
            //   double centerLaserVal = s_Swerve.getCenterLaserValue();
            //   if (s_Swerve.getlimelightLowestYValue() > 195) {
            //     centerLaserVal = 15;
            //   }
            //   ySpeed = yController.calculate(centerLaserVal); 
            // }
            // else {
            //   ySpeed = 0;
            // }
            double minLidarReading = s_Swerve.getCenterLaserValue();
            if (s_Swerve.getLeftLaserValue() < minLidarReading) {
              minLidarReading = s_Swerve.getLeftLaserValue();
            }
            if (s_Swerve.getRightLaserValue() < minLidarReading) {
              minLidarReading = s_Swerve.getRightLaserValue();
            }
            if (minLidarReading < 32) {
              ySpeed = 0.75;
            }
            translation = new Translation2d(-ySpeed, xSpeed);
            
            SmartDashboard.putNumber("rSpeed", rSpeed);
            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("ySpeed", ySpeed);
            s_Swerve.drive(translation, rSpeed, false, true);
            
            // if ((minLidarReading > 31 && minLidarReading < 43.5 - (ySpeed * 10)) && xController.atSetpoint() && currentArmError < 1.75 && currentTarget == Constants.ArmConstants.grabCubePosition) {             
              
            //   grabState++;
            //   s_Swerve.stop();                         
            // }    
            if (xController.atSetpoint() && yMeasurement < 6) {
              grabState++;
              yController.setPID(0.05, Constants.Swerve.autoGrabUprightConeLidarYI, Constants.Swerve.autoGrabUprightConeLidarYD);
              yController.setSetpoint(Constants.Swerve.autoGrabUprightConeLidarYTarget);
              yController.setTolerance(Constants.Swerve.autoGrabUprightConeLidarYTolerance);
              yController.setSetpoint(34);
              yController.setTolerance(4);
            }                   
        }
        else {
          s_Swerve.stop();
        }
      break;
      case 1:
        if (currentArmError < 4) {
          if (currentTarget != Constants.ArmConstants.grabCubePosition) {
            currentTarget = Constants.ArmConstants.grabCubePosition;
            currentArmError = 20;
          }
        }
        degrees =s_Swerve.getYaw().getDegrees();
        if (rController.getSetpoint() > 160 && degrees < 0) {
          degrees = 360 + degrees;
        }
        else if (rController.getSetpoint() < -160 && degrees > 0) {
          degrees = degrees - 360;
        }
        //var boundingBoxXY = s_Swerve.getBoundingBoxX();
        rSpeed = rController.calculate(degrees);   
        double minLidarReading = s_Swerve.getCenterLaserValue();
        if (s_Swerve.getLeftLaserValue() < minLidarReading) {
          minLidarReading = s_Swerve.getLeftLaserValue();
        }
        if (s_Swerve.getRightLaserValue() < minLidarReading) {
          minLidarReading = s_Swerve.getRightLaserValue();
        }

        ySpeed = yController.calculate(minLidarReading); 
        
        xSpeed = xController.calculate(s_Swerve.getLimelightX());
        translation = new Translation2d(-ySpeed, xSpeed); //-ySpeed
        SmartDashboard.putNumber("rSpeed", rSpeed);
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        
        s_Swerve.drive(translation, rSpeed, false, true);
        if (yController.atSetpoint() && currentArmError < 2) {
          isInPositionCnt++;
          if (isInPositionCnt > 5) {
            grabState++;
          }
        }
        else {
          isInPositionCnt = 0;
        }
      break;
      case 2:
          s_Arm.closeClaw();
          waitCnt = 0;
          grabState++;
      break;
      case 3:
          waitCnt++;
          if (waitCnt > 10) {
            currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
            grabState++;
          }
      break;
      case 4:
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
