// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class AutoGrabUprightCone extends CommandBase {
  int grabState = 0;
  Arm s_Arm;
  SwerveDrive s_Swerve;
  PIDController xController = new PIDController(Constants.Swerve.autoDriveConePickupXP, Constants.Swerve.autoDriveConePickupXI, Constants.Swerve.autoDriveConePickupXD);
  PIDController yController = new PIDController(Constants.Swerve.autoDriveConePickupYP, Constants.Swerve.autoDriveConePickupYI, Constants.Swerve.autoDriveConePickupYD);
  PIDController rController = new PIDController(Constants.Swerve.autoDriveConePickupRP, Constants.Swerve.autoDriveConePickupRI, Constants.Swerve.autoDriveConePickupRD);
  ArmPosition currentTarget;
  double rSpeed = 0;
  double xSpeed = 0;
  double ySpeed = 0;
  Translation2d translation;
  double degrees = 0;
  int waitCnt = 0;
  boolean isDone = false;
  /** Creates a new AutoGrabUprightCone. */
  public AutoGrabUprightCone(Arm s_Arm, SwerveDrive s_Swerve) {
    this.s_Arm = s_Arm;
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve, s_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    s_Swerve.setLimelightPipeline(1);
    s_Swerve.setLimelightLED(false);
    xController.setSetpoint(150);
    xController.setTolerance(Constants.Swerve.autoGrabUprightConeXLimelightTolerance);
    yController.setSetpoint(Constants.Swerve.autoGrabUprightConeYLimelightTarget);
    yController.setTolerance(Constants.Swerve.autoGrabUprightConeYLimelightTolerance);
    rController.setSetpoint(s_Swerve.getYaw().getDegrees());
    rController.setTolerance(Constants.Swerve.autoGrabUprightConeRTolerance);
    currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
    grabState = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentArmError = s_Arm.MoveArm(currentTarget);
    switch(grabState) {
      case 0:
        degrees =s_Swerve.getYaw().getDegrees();
        // if (degrees < 0) {
        //   degrees += 360;
        // }
        var boundingBoxXY = s_Swerve.getBoundingBoxX();
        rSpeed = rController.calculate(degrees);
        xSpeed = xController.calculate(boundingBoxXY[1]); 
        ySpeed = yController.calculate(s_Swerve.getLimelightY()); 
        
        translation = new Translation2d(-ySpeed, xSpeed);
        
        
        s_Swerve.drive(translation, rSpeed, false, true);
        if (s_Swerve.getLimelightY() < Constants.Swerve.autoGrabUprightConeYSwitchToLidar && xController.atSetpoint()) {
            if (currentArmError < 5) {
              grabState++;
            }
        }
      break;
      case 1:
        currentTarget = Constants.ArmConstants.grabUprightConePosition;
        //xController.setPID(Constants.Swerve.autoGrabUprightConeLidarXP, Constants.Swerve.autoGrabUprightConeLidarXI, Constants.Swerve.autoGrabUprightConeLidarXD);
        yController.setPID(Constants.Swerve.autoGrabUprightConeLidarYP, Constants.Swerve.autoGrabUprightConeLidarYI, Constants.Swerve.autoGrabUprightConeLidarYD);
        yController.setSetpoint(Constants.Swerve.autoGrabUprightConeLidarYTarget);
        yController.setTolerance(Constants.Swerve.autoGrabUprightConeLidarYTolerance);
        grabState++;
      break;
      case 2:
        degrees =s_Swerve.getYaw().getDegrees();
        // if (degrees < 0) {
        //   degrees += 360;
        // }
        //var boundingBoxXY = s_Swerve.getBoundingBoxX();
        rSpeed = rController.calculate(degrees);       
        ySpeed = yController.calculate(s_Swerve.getCenterLaserValue()); 
        if (s_Swerve.getLeftLaserValue() - s_Swerve.getCenterLaserValue() < -10) {
          xSpeed = 0.5;
        }
        else if (s_Swerve.getRightLaserValue() - s_Swerve.getCenterLaserValue() < -10) {
          xSpeed = -0.5;
        }
        else if (Math.abs(s_Swerve.getLeftLaserValue() - s_Swerve.getCenterLaserValue()) < 8) {
          xSpeed = 0.25;
        }
        else if (Math.abs(s_Swerve.getRightLaserValue() - s_Swerve.getCenterLaserValue()) < 8) {
          xSpeed = -0.25;
        }
        else {
          xSpeed = 0;
        }
        translation = new Translation2d(-ySpeed, xSpeed); 
        
        
        s_Swerve.drive(translation, rSpeed, false, true);
        if (yController.atSetpoint() && currentArmError < 2) {
          grabState++;
        }
      break;
      case 3:
        s_Swerve.stop();
        s_Arm.closeClaw();
        grabState++;
      break;
      case 4:
        waitCnt++;
        if (waitCnt > 10) {
          grabState++;
        }
      break;
      case 5:
        currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
        if (currentArmError < 4) {
          grabState++;
        }
      break;
      case 6:
        currentTarget = Constants.ArmConstants.restPosition;
        s_Arm.setLEDs(0, 255, 0);
        isDone = true;
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.setHeadlights(false);
    s_Swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
