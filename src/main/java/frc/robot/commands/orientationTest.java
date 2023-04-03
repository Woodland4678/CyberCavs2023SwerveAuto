// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class orientationTest extends CommandBase {
  SwerveDrive s_Swerve;
  double angle = 0;
  Arm s_Arm;
  ArmPosition currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
  double currentArmError;
  double[] orientationReadings = new double[100];
  int orientationReadingsIndex = 0;
  PIDController xController = new PIDController(0.012, Constants.Swerve.autoDriveConePickupXI, Constants.Swerve.autoDriveConePickupXD);
  PIDController yController = new PIDController(0.065,0,0);
  PIDController rController = new PIDController(Constants.Swerve.autoDriveConePickupRP, Constants.Swerve.autoDriveConePickupRI, Constants.Swerve.autoDriveConePickupRD);
  boolean finalOrientationSet = false;
  double angleToUse;
  boolean isInPickup = false;
  int grabState = 0;
  int grabCnt = 0;
  /** Creates a new orientationTest. */
  public orientationTest(SwerveDrive s_Swerve, Arm s_Arm) {
    this.s_Swerve = s_Swerve;
    this.s_Arm = s_Arm;
    currentArmError = 0;
    addRequirements(s_Swerve, s_Arm);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    grabCnt = 0;
    grabState = 0;
    isInPickup = false;
    //s_Swerve.setHeadlights(true);
    s_Swerve.limelightDown();
    s_Swerve.setLimelightPipeline(6);
    s_Swerve.setLimelightLED(false);
    yController.reset();
    yController.setSetpoint(15);
    xController.setSetpoint(160);
    rController.setSetpoint(s_Swerve.getYaw().getDegrees());
    xController.setTolerance(12);
    yController.setTolerance(1);
    finalOrientationSet = false;
    if (s_Arm.getCurrentArmPosition() == Constants.ArmConstants.pickupPosition) {
      currentTarget = Constants.ArmConstants.pickupPosition;
    }
    else {
      currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
    }
    orientationReadingsIndex = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentArmError = s_Arm.MoveArm(currentTarget);
    switch (grabState) {
      case 0:
        if (currentArmError < 4) {
          currentTarget = Constants.ArmConstants.pickupPosition;      
        }
        var coneAngle = s_Swerve.getConeAngle();
        // currentTarget.wristRollTarget = -coneAngle;
        var boundingBoxXY = s_Swerve.getBoundingBoxX();
        var boundingBox = s_Swerve.getBoundingBoxMinMaxX();        
        
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
          currentTarget.wristRollTarget = finalOrientation;
          //Constants.ArmConstants.grabConePosition.wristRollTarget = finalOrientation;
        }
        // angle += 0.25;
        // if (angle > 180) {
        //   angle = -180;
        // }
        if (!finalOrientationSet) {
          angleToUse = coneAngle;
        }
        else {
          angleToUse = Constants.ArmConstants.pickupPosition.wristRollTarget;
        }
        double value = SwerveDrive.GetConePosition(boundingBox[0], boundingBox[1], angleToUse).GetY(); //s_Swerve.getConeAngle()
        SmartDashboard.putNumber("Wilms distance to cone", value);
        SmartDashboard.putNumber("Wilms set cone angle", coneAngle); // s_Swerve.getConeAngle()
        SmartDashboard.putNumber("Wilms Final Orientation", currentTarget.wristRollTarget);
        double ySpeed = yController.calculate(value);
        double xSpeed = xController.calculate(boundingBoxXY[1]);
        double rSpeed = rController.calculate(s_Swerve.getYaw().getDegrees());
        if (yController.atSetpoint() && xController.atSetpoint() && finalOrientationSet) {
          s_Swerve.stop();
          if (Math.abs(s_Arm.getCurrentWristRollPosition() - currentTarget.wristRollTarget) < 3) {
            grabState++;
          }
        }
        else {
          Translation2d translation = new Translation2d(-ySpeed, xSpeed);
          s_Swerve.drive(translation, rSpeed, false,true);
        }
      break;
      case 1:
        var currentTargetTemp = Constants.ArmConstants.grabConePosition;
        currentTargetTemp.wristRollTarget = currentTarget.wristRollTarget;
        currentTarget = currentTargetTemp;
        grabState++;
      break;
      case 2:
        if (currentArmError < 3) {
          grabCnt++;
          if (grabCnt > 10) {
            grabState++;
            grabCnt = 0;
          }
        }
        else {
          grabCnt = 0;
        }
      break;
      case 3:
        s_Arm.closeClaw();
        grabCnt++;
        if (grabCnt > 25) {
          grabState++;
        }
      break;
      case 4:
        currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
      break;
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
