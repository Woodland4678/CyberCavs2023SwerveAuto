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
  ArmPosition currentTarget;
  int grabState = 0;
  double degrees = 0;
  Translation2d translation;
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
    s_Arm.openClaw();
    rController.reset();
    xController.reset();
    yController.reset();
   // s_Swerve.setHeadlights(true);
    isDone = false;
    waitCnt = 0;
    rSpeed = 0;
    xSpeed = 0;
    ySpeed = 0;
    s_Swerve.limelightDown();
    s_Swerve.setLimelightPipeline(0);
    s_Swerve.setLimelightLED(false);
    xController.setSetpoint(0);
    xController.setTolerance(Constants.Swerve.autoDriveCubePickupXTolerance);
    yController.setSetpoint(Constants.Swerve.autoGrabCubeLidarTarget);
    yController.setTolerance(Constants.Swerve.autoGrabCubeYTolerance);
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
          if (currentArmError < 4) {
            currentTarget = Constants.ArmConstants.grabCubePosition;
          }
          if (s_Swerve.limelightHasTarget() == 1) {
            degrees = s_Swerve.getYaw().getDegrees();
            // if (degrees < 0) {
            //   degrees += 360;
            // }
            //var boundingBoxXY = s_Swerve.getBoundingBoxX();
            rSpeed = rController.calculate(degrees);
            xSpeed = xController.calculate(s_Swerve.getLimelightX()); 
            if (Math.abs(s_Swerve.getLimelightX()) < Constants.Swerve.autoGrabCubeEnableY) { //don't move forward until x is close enough
              ySpeed = yController.calculate(s_Swerve.getCenterLaserValue()); 
            }
            else {
              ySpeed = 0;
            }
            
            translation = new Translation2d(-ySpeed, xSpeed);
            
            
            s_Swerve.drive(translation, rSpeed, false, true);
            if (yController.atSetpoint() && xController.atSetpoint()) {
              grabState++;
              s_Swerve.stop();
            }            
        }
        else {
          s_Swerve.stop();
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
