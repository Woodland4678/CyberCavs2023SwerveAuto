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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class AutoPickup extends CommandBase {
  Arm s_Arm;
  SwerveDrive s_Swerve;
  PIDController xController = new PIDController(Constants.Swerve.autoDriveXP, Constants.Swerve.autoDriveXI, Constants.Swerve.autoDriveXD);
  PIDController yController = new PIDController(Constants.Swerve.autoDriveYP, Constants.Swerve.autoDriveYI, Constants.Swerve.autoDriveYD);
  PIDController rController = new PIDController(Constants.Swerve.autoDriveRP, Constants.Swerve.autoDriveRI, Constants.Swerve.autoDriveRD);
  boolean isDone = false;
  int isInPosCnt = 0;
  double limelightYTarget = 94;
  double[] orientationReadings = new double[100];
  int orientationReadingsIndex = 0;
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
    if (s_Arm.getGamePieceMode() == Constants.ArmConstants.coneMode) {
      limelightYTarget = Constants.Swerve.coneAutoDriveYTarget;
    }
    else {
      limelightYTarget = Constants.Swerve.cubeAutoDriveYTarget;
    }
    xController.setSetpoint(0);
    rController.setSetpoint(s_Swerve.getYaw().getDegrees());
    yController.setSetpoint(limelightYTarget);
    xController.setTolerance(1);
    yController.setTolerance(0.5);
    s_Swerve.setLimelightPipeline(1);
    isDone = false;
    isInPosCnt = 0;
    s_Swerve.limelightDown();
    orientationReadingsIndex = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    var armPosError = s_Arm.MoveArm(Constants.ArmConstants.pickupPosition);    
    var degrees =s_Swerve.getYaw().getDegrees();
    //if (degrees < 0) {
     // degrees += 360;
   // }
    var boundingBoxXY = s_Swerve.getBoundingBoxX()[0];
    var coneAngle = s_Swerve.getConeAngle();
    var rSpeed = rController.calculate(degrees);
    var xSpeed = xController.calculate(s_Swerve.getLimelightX());
    var ySpeed = yController.calculate(boundingBoxXY);

    if ((boundingBoxXY < 80 && boundingBoxXY > 50) && s_Swerve.isConeFound() == 1) {
      if (orientationReadingsIndex < 100) {
        orientationReadings[orientationReadingsIndex] = -coneAngle;
        orientationReadingsIndex++;
      }
    }
    if (boundingBoxXY > 85) {
      Arrays.sort(orientationReadings, 0, orientationReadingsIndex);
      Constants.ArmConstants.pickupPosition.wristRollTarget = orientationReadings[orientationReadingsIndex / 2];
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
    
    s_Swerve.drive(translation, rSpeed, false, true);

    if (armPosError < 3 && Math.abs(boundingBoxXY - limelightYTarget) < 2) {
      isInPosCnt++;
    }
    if (isInPosCnt > 10) {
      s_Arm.ClawClose();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
