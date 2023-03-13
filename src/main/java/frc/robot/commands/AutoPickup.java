// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;

import javax.print.attribute.standard.OrientationRequested;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class AutoPickup extends CommandBase {
  Arm s_Arm;
  SwerveDrive s_Swerve;
  PIDController xController = new PIDController(Constants.Swerve.autoDriveConePickupXP, Constants.Swerve.autoDriveConePickupXI, Constants.Swerve.autoDriveConePickupXD);
  PIDController yController = new PIDController(Constants.Swerve.autoDriveConePickupYP, Constants.Swerve.autoDriveConePickupYI, Constants.Swerve.autoDriveConePickupYD);
  PIDController rController = new PIDController(Constants.Swerve.autoDriveConePickupRP, Constants.Swerve.autoDriveConePickupRI, Constants.Swerve.autoDriveConePickupRD);
  boolean isDone = false;
  int isInPosCnt = 0;
  double limelightYTarget = 99;
  double[] orientationReadings = new double[100];
  int orientationReadingsIndex = 0;
  double currentArmError = 0;
  int isDoneCnt = 0;
  boolean pickupPosSet = false;
  ArmPosition currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
  /** Creates a new AutoPickup. */
  public AutoPickup(Arm s_Arm, SwerveDrive s_Swerve) {
    this.s_Arm = s_Arm;
    this.s_Swerve = s_Swerve;
    
    addRequirements(s_Arm, s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    rController.reset();
    pickupPosSet = false;
    //s_Swerve.setHeadlights(true);
    s_Swerve.limelightDown();
    isDoneCnt = 0;
    if (s_Arm.getGamePieceMode() == Constants.ArmConstants.coneMode) {
      limelightYTarget = Constants.Swerve.coneAutoDriveYTarget;
    }
    else {
      limelightYTarget = Constants.Swerve.cubeAutoDriveYTarget;
    }
    xController.setSetpoint(3);
    rController.setSetpoint(s_Swerve.getYaw().getDegrees());
    yController.setSetpoint(limelightYTarget);
    xController.setTolerance(5);
    yController.setTolerance(12);
    if (s_Arm.gamePieceMode == Constants.ArmConstants.coneMode) {
      s_Swerve.setLimelightPipeline(6); //1
    }
    else {
      s_Swerve.setLimelightPipeline(2);
    }
    isDone = false;
    isInPosCnt = 0;
    orientationReadingsIndex = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.stop();
    if (currentArmError < 4 && !pickupPosSet) {
      currentTarget = Constants.ArmConstants.pickupPosition;
      pickupPosSet = true;
    }
    var degrees =s_Swerve.getYaw().getDegrees();
    //if (degrees < 0) {
     // degrees += 360;
   // }
    var boundingBoxXY = s_Swerve.getBoundingBoxX()[0];
    var coneAngle = s_Swerve.getConeAngle();
    var rSpeed = rController.calculate(degrees);
    var xSpeed = xController.calculate(s_Swerve.getLimelightX());
    var ySpeed = yController.calculate(boundingBoxXY);
    if (s_Arm.getGamePieceMode() == Constants.ArmConstants.coneMode) {
     // Constants.ArmConstants.pickupPosition.wristRollTarget = -coneAngle;



      if ((boundingBoxXY < 80 && boundingBoxXY > 50) && s_Swerve.isConeFound() == 1) {
        if (orientationReadingsIndex < 100) {
          orientationReadings[orientationReadingsIndex] = -coneAngle;
          orientationReadingsIndex++;
        }
      }
      if (boundingBoxXY > 85) {
        Arrays.sort(orientationReadings, 0, orientationReadingsIndex);
        Constants.ArmConstants.pickupPosition.wristRollTarget = orientationReadings[orientationReadingsIndex / 2];
        Constants.ArmConstants.grabConePosition.wristRollTarget = orientationReadings[orientationReadingsIndex / 2];
      }




    }
    // if (s_Swerve.isConeFound() == 1) {
    //   Constants.ArmConstants.pickupPosition.wristRollTarget = -coneAngle;
    // }
    Translation2d translation = new Translation2d(ySpeed, xSpeed);
    SmartDashboard.putNumber(
                "xSpeed",xSpeed);
    SmartDashboard.putNumber(
                  "ySpeed",ySpeed);
    // SmartDashboard.putNumber(
    //               "rSpeed",0);
    //SmartDashboard.putNumber(
    //              "coneAngle",coneAngle);
    
    

    if (currentArmError < 3 && yController.atSetpoint() && xController.atSetpoint()) {
      isInPosCnt++;
    }
    else {
      currentArmError = s_Arm.MoveArm(currentTarget);   
    }
    if (isInPosCnt > 10) {
      s_Swerve.stop();
      if (s_Arm.gamePieceMode == Constants.ArmConstants.coneMode) {
        currentArmError = s_Arm.MoveArm(Constants.ArmConstants.grabConePosition);
      }
      else {
        currentArmError = s_Arm.MoveArm(Constants.ArmConstants.grabCubePosition);
      }
      if (Math.abs(currentArmError) < 3) {
        s_Arm.closeClaw();
        isDoneCnt++;
        if (isDoneCnt > 15) {
          isDone = true;
        }
      }
    }
    else {
      s_Swerve.drive(translation, rSpeed, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Arm.MoveArm(Constants.ArmConstants.pickupToRestIntermediatePosition);
    s_Swerve.stop();
    //s_Swerve.setHeadlights(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}