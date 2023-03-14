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
  double limelightYTarget = 120;
  double[] orientationReadings = new double[100];
  int orientationReadingsIndex = 0;
  double currentArmError = 0;
  int isDoneCnt = 0;
  boolean pickupPosSet = false;
  ArmPosition currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
  double degrees0Y = 153.5;
  double degrees0X = 170;
  double degrees0D = 0;
  double degrees0W = -20;

  double degrees90Y = 130;
  double degrees90X = 186;
  double degrees90D = 90;
  double degrees90W = 0;

  double degrees180Y = 120;
  double degrees180X = 177;
  double degrees180D = 180;
  double degrees180W = 20;

  double degreesNeg90Y = 126.5;
  double degreesNeg90X = 169;
  double degreesNeg90D = -90;
  double degreesNeg90W = 0;

  boolean finalOrientationSet = false;
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
    finalOrientationSet = false;
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
    xController.setSetpoint(170);
    rController.setSetpoint(s_Swerve.getYaw().getDegrees());
    yController.setSetpoint(limelightYTarget);
    xController.setTolerance(5);
    yController.setTolerance(7);
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
  private double linearInterpolateX(double myAngle, double angle1, double x1, double angle2, double x2) {
    return (x1 + (myAngle - angle1) * ((x2 - x1)/(angle2 - angle1)));
  }
  private double linearInterpolateY(double myAngle, double angle1, double y1, double angle2, double y2) {
    return (y1 + (myAngle - angle1) * ((y2 - y1)/(angle2 - angle1)));
  }
  private double linearInterpolateW(double myAngle, double angle1, double w1, double angle2, double w2) {
    return (w1 + (myAngle - angle1) * ((w2 - w1)/(angle2 - angle1)));
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
    var boundingBoxXY = s_Swerve.getBoundingBoxX();
    var coneAngle = s_Swerve.getConeAngle();
    var rSpeed = rController.calculate(degrees);
    var xSpeed = xController.calculate(boundingBoxXY[1]); //s_Swerve.getLimelightX();
    var ySpeed = yController.calculate(boundingBoxXY[0]);
    if (s_Arm.getGamePieceMode() == Constants.ArmConstants.coneMode) {
     // Constants.ArmConstants.pickupPosition.wristRollTarget = -coneAngle;



      if ((boundingBoxXY[0] < 80 && boundingBoxXY[0] > 50) && s_Swerve.isConeFound() == 1) {
        if (orientationReadingsIndex < 100) {
          orientationReadings[orientationReadingsIndex] = -coneAngle;
          orientationReadingsIndex++;
        }
      }
      if (boundingBoxXY[0] > 85 && !finalOrientationSet) {
        finalOrientationSet = true;
        Arrays.sort(orientationReadings, 0, orientationReadingsIndex);
        double finalOrientation = orientationReadings[orientationReadingsIndex / 2];
        Constants.ArmConstants.pickupPosition.wristRollTarget = finalOrientation;
        Constants.ArmConstants.grabConePosition.wristRollTarget = finalOrientation;
        double newXTarget = xController.getSetpoint();
        double newYTarget = yController.getSetpoint();
        double newWristTarget = 0;
        finalOrientation *= -1;
        if (finalOrientation < 0 && finalOrientation > -90) {
          newXTarget = linearInterpolateX(finalOrientation, degreesNeg90D, degreesNeg90X, degrees0D, degrees0X);
          newYTarget = linearInterpolateY(finalOrientation, degreesNeg90D, degreesNeg90Y, degrees0D, degrees0Y);
          newWristTarget = linearInterpolateW(finalOrientation, degreesNeg90D, degreesNeg90W, degrees0D, degrees0W);
        }
        else if (finalOrientation < -90 && finalOrientation > -180) {
          newXTarget = linearInterpolateX(finalOrientation, degrees180D, degrees180X, degreesNeg90D, degreesNeg90X);
          newYTarget = linearInterpolateY(finalOrientation, degrees180D, degrees180Y, degreesNeg90D, degreesNeg90Y);
          newWristTarget = linearInterpolateW(finalOrientation, degrees180D, degrees180W, degreesNeg90D, degreesNeg90W);
        }
        else if (finalOrientation > 0 && finalOrientation < 90) {
          newXTarget = linearInterpolateX(finalOrientation, degrees90D, degrees90X, degrees0D, degrees0X);
          newYTarget = linearInterpolateY(finalOrientation, degrees90D, degrees90Y, degrees0D, degrees0Y);
          newWristTarget = linearInterpolateW(finalOrientation, degrees90D, degrees90W, degrees0D, degrees0W);
        }
        else if (finalOrientation > 90 && finalOrientation < 180) {
          newXTarget = linearInterpolateX(finalOrientation, degrees180D, degrees180X, degrees90D, degrees90X);
          newYTarget = linearInterpolateY(finalOrientation, degrees180D, degrees180Y, degrees90D, degrees90Y);
          newWristTarget = linearInterpolateW(finalOrientation, degrees180D, degrees180W, degrees90D, degrees90W);
        }
        SmartDashboard.putNumber("auto pickup tipped cone new x target", newXTarget);
        SmartDashboard.putNumber("auto pickup tipped cone new y target", newYTarget);
        SmartDashboard.putNumber("auto pickup tipped cone final orientation", finalOrientation);
        Constants.ArmConstants.grabConePosition.wristPitchTarget += newWristTarget;
        xController.setSetpoint(newXTarget);
        yController.setSetpoint(newYTarget);
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
    if (isInPosCnt > 10 && false) { //TODO remove this false
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
        if (isDoneCnt > 40) {
          isDone = true;
        }
      }
    }
    else {
      //s_Swerve.drive(translation, rSpeed, false, true);
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