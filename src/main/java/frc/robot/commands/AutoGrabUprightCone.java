// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.LEDModes;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class AutoGrabUprightCone extends CommandBase {
  int grabState = 0;
  Arm s_Arm;
  SwerveDrive s_Swerve;
  PIDController xController = new PIDController(Constants.Swerve.autoDriveConePickupXP, Constants.Swerve.autoDriveConePickupXI, Constants.Swerve.autoDriveConePickupXD);
  PIDController yController = new PIDController(Constants.Swerve.autoDriveConePickupYP, Constants.Swerve.autoDriveConePickupYI, Constants.Swerve.autoDriveConePickupYD);
  PIDController rController = new PIDController(0.1, 0.0, 0.005); //0.07 0.0001 0.005
  ArmPosition currentTarget;
  double rSpeed = 0;
  double xSpeed = 0;
  double ySpeed = 0;
  Translation2d translation;
  double degrees = 0;
  int waitCnt = 0;
  boolean isDone = false;
  int isInPositionCnt = 0;
  double rTarget = 0;
  boolean autonomousForceStop = false;
  boolean doesNeedToBalance = false;
  double initialCenterLidarValue = 0;
  /** Creates a new AutoGrabUprightCone. */
  public AutoGrabUprightCone(Arm s_Arm, SwerveDrive s_Swerve, double rTarget, boolean doesNeedToBalance) {
    this.s_Arm = s_Arm;
    this.rTarget = rTarget;
    this.s_Swerve = s_Swerve;
    this.doesNeedToBalance = doesNeedToBalance;
    addRequirements(s_Swerve, s_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rController.setP(0.1);
    rController.setD(0.005);
    autonomousForceStop =false;
    isInPositionCnt = 0;
    s_Arm.openClaw();
    yController.setPID(Constants.Swerve.autoDriveConePickupYP, Constants.Swerve.autoDriveConePickupYI, Constants.Swerve.autoDriveConePickupYD);
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
    s_Swerve.setLimelightPipeline(6); // was 1
    s_Swerve.setLimelightLED(false);
    xController.setSetpoint(160);
    xController.setTolerance(Constants.Swerve.autoGrabUprightConeXLimelightTolerance);
    yController.setSetpoint(Constants.Swerve.autoGrabUprightConeYLimelightTarget);
    yController.setTolerance(Constants.Swerve.autoGrabUprightConeYLimelightTolerance);    
    if (DriverStation.isAutonomous()) {
      rController.setSetpoint(rTarget);
    }
    else {
      rController.setSetpoint(s_Swerve.getYaw().getDegrees());
    }
    initialCenterLidarValue = 0;
    rController.setTolerance(Constants.Swerve.autoGrabUprightConeRTolerance);
    currentTarget = Constants.ArmConstants.pickupToRestIntermediatePositionAuto;
    grabState = 0;    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // var testPIDs = s_Swerve.getRotationPID();
    // rController.setPID(testPIDs[0], testPIDs[1], testPIDs[2]);
    var fieldPose = s_Swerve.getPose();
    //if we're in auto and getting close to the middle of the field then something has gone wrong and we should stop
    if (DriverStation.isAutonomous() && fieldPose.getTranslation().getX() > 7.1 && !autonomousForceStop) {
      grabState = 3;
      autonomousForceStop = true;
    }
    double currentArmError = s_Arm.MoveArm(currentTarget);
    switch(grabState) {

      case 0:
        
        degrees =s_Swerve.getYaw().getDegrees();
        if (rController.getSetpoint() > 160 && degrees < 0) {
          degrees = 360 + degrees;
        }
        else if (rController.getSetpoint() < -160 && degrees > 0) {
          degrees = degrees - 360;
        }
        
        var boundingBoxXY = s_Swerve.getBoundingBoxX();
        rSpeed = rController.calculate(degrees);
        xSpeed = xController.calculate(boundingBoxXY[1]); 
        ySpeed = yController.calculate(s_Swerve.getLimelightY()); 
        if (rController.getPositionError() < 3.5) {
          rController.setP(0.065); //0.025
          rController.setD(0);
        }
        else {
          rController.setP(0.1);
          rController.setD(0.005);
        }
        if (s_Swerve.getLimelightY() < yController.getSetpoint()) {
          ySpeed = 0;
        }
        if (rController.getPositionError() > 3.75) {
          //xSpeed = xSpeed * 0.8;
          ySpeed = ySpeed * 0.8;
        }
        SmartDashboard.putNumber("rSpeed", rSpeed);
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        translation = new Translation2d(-ySpeed, xSpeed); //-yspeed
        
        if (s_Swerve.limelightHasTarget() == 1) {
          s_Swerve.drive(translation, rSpeed, false, true);
        }
        SmartDashboard.putBoolean(
          "upright cone limelight score y controller",yController.atSetpoint());
        SmartDashboard.putBoolean(
                  "upright cone limelight x controller",xController.atSetpoint());
        SmartDashboard.putBoolean(
                  "upright cone limelight r controller",rController.atSetpoint());
        if (xController.atSetpoint() && rController.atSetpoint() && (yController.atSetpoint() || s_Swerve.getLimelightY() < yController.getSetpoint() + 3)) {
            if (currentArmError < 3) {
              grabState++;
              waitCnt = 0;
            }
        }
      break;
      case 1:
        currentTarget = Constants.ArmConstants.grabUprightConePosition;
        initialCenterLidarValue = s_Swerve.getCenterLaserValue();
        //xController.setPID(Constants.Swerve.autoGrabUprightConeLidarXP, Constants.Swerve.autoGrabUprightConeLidarXI, Constants.Swerve.autoGrabUprightConeLidarXD);
        yController.setPID(Constants.Swerve.autoGrabUprightConeLidarYP, Constants.Swerve.autoGrabUprightConeLidarYI, Constants.Swerve.autoGrabUprightConeLidarYD);
        yController.setSetpoint(Constants.Swerve.autoGrabUprightConeLidarYTarget);
        yController.setTolerance(Constants.Swerve.autoGrabUprightConeLidarYTolerance);
        //s_Swerve.setModulesStraight();
        //waitCnt++;
        //if (waitCnt > 5) {
          grabState++;
        //}
      break;
      case 2:
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
        if (s_Swerve.getLeftLaserValue() - s_Swerve.getCenterLaserValue() < -20) {
          xSpeed = 0.5;
          ySpeed = ySpeed * 0.7;
        }
        else if (s_Swerve.getRightLaserValue() - s_Swerve.getCenterLaserValue() < -20) {
          xSpeed = -0.5;
          ySpeed = ySpeed * 0.7;
        }
        else if (Math.abs(s_Swerve.getLeftLaserValue() - s_Swerve.getCenterLaserValue()) < 12) {
          xSpeed = 0.25;
        }
        else if (Math.abs(s_Swerve.getRightLaserValue() - s_Swerve.getCenterLaserValue()) < 12) {
          xSpeed = -0.25;
        }
        else {
          xSpeed = 0;
        }
        if (rController.getPositionError()> 4) {
          //xSpeed = xSpeed * 0.5;
          ySpeed = ySpeed * 0.5;
        }
        translation = new Translation2d(-ySpeed, xSpeed); //-ySpeed
        SmartDashboard.putNumber("rSpeed", rSpeed);
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        
        s_Swerve.drive(translation, rSpeed, false, true);
        if (yController.atSetpoint() && currentArmError < 1.5) {
          isInPositionCnt++;
          if (isInPositionCnt > 3) {
            grabState++;
          }
        }
        else {
          isInPositionCnt = 0;
        }
      break;
      case 3:
        s_Swerve.stop();
        if (!autonomousForceStop) {
          s_Arm.closeClaw();
          waitCnt = 0;
        }
        grabState++;
      break;
      case 4:
        waitCnt++;
        if (waitCnt > 17) {
          grabState++;
        }
      break;
      case 5:
        if (DriverStation.isAutonomous()) {
          currentTarget = Constants.ArmConstants.pickupToRestIntermediatePositionAuto;
        }
        else {
          currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
        }
        if (currentArmError < 5) {
          grabState++;
        }
      break;
      case 6:
        if (DriverStation.isAutonomous()) {
          currentTarget = Constants.ArmConstants.restPositionAuto;
        }
        else {
          currentTarget = Constants.ArmConstants.restPosition;
        }
        
        
        if (!autonomousForceStop || doesNeedToBalance) { //only return isdone = true if we didn't have to force stop auto
          isDone = true;
          s_Arm.setLEDMode(LEDModes.BLINKGREEN);
        }
        else if (!doesNeedToBalance) {
          grabState = 8;
        }
      break;
      case 7:
        PathPlannerTrajectory goBalance = PathPlanner.generatePath(
          new PathConstraints(2, 2), 
          new PathPoint(new Translation2d(s_Swerve.getPose().getX(), s_Swerve.getPose().getY()), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(s_Swerve.getYaw().getDegrees())), // position, heading
          new PathPoint(new Translation2d(5.56, 2.72), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(180)), // position, heading
          new PathPoint(new Translation2d(3.46, 2.72), Rotation2d.fromDegrees(180),  Rotation2d.fromDegrees(180))
        );
        new RunCommand(() -> s_Swerve.followTrajectoryCommand(PathPlannerTrajectory.transformTrajectoryForAlliance(goBalance, DriverStation.getAlliance()), true),
        s_Swerve);
        new RunCommand(() -> new AutoBalance(s_Swerve), s_Swerve);
        grabState++;
      break;
      case 8:

      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.setHeadlights(false);
    s_Swerve.stop();
    s_Arm.setLEDMode(LEDModes.BLINKGREEN);
    s_Swerve.limelightUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
