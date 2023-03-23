// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.management.OperatingSystemMXBean;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
  Arm s_Arm;
  ArmPosition targetPos;
  double currentArmError = 0;
  boolean doneIntermediateMovement = false;
  ArmPosition currentTarget;
  CommandXboxController operatorJoystick;
  boolean armOkayToMove = true;
  boolean isDone = false;
  int isDoneCnt = 0;
  int clawClosedCnt = 0;
  ArmPosition originalTarget;
  /** Creates a new MoveArm. */
  public MoveArm(Arm s_Arm, ArmPosition targetPos, CommandXboxController operatorJoystick) {
    this.s_Arm = s_Arm;
    this.targetPos = targetPos;
    this.operatorJoystick = operatorJoystick;
    originalTarget = targetPos;
    addRequirements(s_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clawClosedCnt = 0;
    isDone = false;
    isDoneCnt = 0;
    armOkayToMove = true;
    //if we're going to pickup position set the target as the intermediate position first
    if (this.targetPos == Constants.ArmConstants.pickupPosition) {
      this.targetPos.wristRollTarget = 0;
     currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition; 
    }
    else if (this.targetPos == Constants.ArmConstants.grabUprightConePosition) {
      currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
      
      if (s_Arm.getGamePieceMode() == Constants.ArmConstants.coneMode) {
        this.targetPos = Constants.ArmConstants.grabUprightConePosition;
      }
      else if (s_Arm.getGamePieceMode() == Constants.ArmConstants.cubeMode) {
        this.targetPos = Constants.ArmConstants.grabCubePosition;
      }
     
    }
    else if ((this.targetPos == Constants.ArmConstants.restPosition || this.targetPos == Constants.ArmConstants.restPositionAuto) && s_Arm.getCurrentXPosition() > 40 && DriverStation.isAutonomous()) {
      currentTarget = Constants.ArmConstants.scoreHighToRestIntermediatePosition;
      
    }
    else if (this.targetPos == Constants.ArmConstants.restPosition) {
      currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
      this.targetPos.wristRollTarget = 90;
    }
    else if (this.targetPos == Constants.ArmConstants.restPositionAuto) {
      currentTarget = Constants.ArmConstants.pickupToRestIntermediatePositionAuto;
    }
    else if (this.targetPos == Constants.ArmConstants.grabFromSingleStationPosition) {
      currentTarget =Constants.ArmConstants.pickupToRestIntermediatePosition;
    }
    else if (this.targetPos == Constants.ArmConstants.scoreConeHighPosition && s_Arm.getGamePieceMode() == Constants.ArmConstants.cubeMode) {
      SmartDashboard.putNumber("IS IT A CUBE WHAT??", s_Arm.getGamePieceMode()) ;
      currentTarget = Constants.ArmConstants.restToScoreHighIntermediatePosition;
      this.targetPos = Constants.ArmConstants.scoreCubeHighPosition;
    }
    else if (this.targetPos == Constants.ArmConstants.scoreConeMediumPosition && s_Arm.getGamePieceMode() == Constants.ArmConstants.coneMode) {
      currentTarget = Constants.ArmConstants.restToScoreMediumIntermediatePosition;
    }
    else if (this.targetPos == Constants.ArmConstants.scoreConeMediumPosition && s_Arm.getGamePieceMode() == Constants.ArmConstants.cubeMode) {
      currentTarget = Constants.ArmConstants.restToScoreMediumIntermediatePosition;
      this.targetPos = Constants.ArmConstants.scoreCubeMediumPosition;
    }
    else if (this.targetPos == Constants.ArmConstants.scoreConeHighPosition) {
      currentTarget = Constants.ArmConstants.restToScoreHighIntermediatePosition;
    }
    else if (this.targetPos == Constants.ArmConstants.scoreLowPosition) {
      currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
    }
    else {
      currentTarget = Constants.ArmConstants.restToScoreHighIntermediatePosition;
    }
    currentArmError = 20;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO remove below code
    /*TEMPORARY CODE REMEMBER TO REMOVE AFTER REVEAL VIDEO */
    // if (operatorJoystick.getPOV() == 180 && currentTarget == Constants.ArmConstants.headTiltForVideoPosition) {
    //   currentTarget.wristRollTarget = -265;
    // }
    if (currentTarget == Constants.ArmConstants.restPosition || currentTarget == Constants.ArmConstants.restPositionAuto) {
      isDoneCnt++;
      if (isDoneCnt > 50) {
        isDone = true;
      }      
    }
    if (currentTarget == Constants.ArmConstants.grabConePosition || currentTarget == Constants.ArmConstants.grabCubePosition || currentTarget == Constants.ArmConstants.grabUprightConePosition) {
      if (s_Arm.getCurrentXPosition() < 15) {
        armOkayToMove = true;
      }
      else {
        armOkayToMove = false;
      }
    }
    if (armOkayToMove) {
      if (currentTarget.isAngleTarget) {
        s_Arm.moveToAngle(currentTarget.xTarget, currentTarget.yTarget); //in this case x and y are shoulder and elbow angles
      }
      else {
        currentArmError = s_Arm.MoveArm(currentTarget);
      }
      if (Math.abs(currentArmError) < 10) { //handles intermediate positions
        currentTarget = targetPos;
        //Manual wrist Roll control (doesn't work too well)
        /*double wristRollSpeed = operatorJoystick.getRawAxis(0);
        if (Math.abs(wristRollSpeed) > 0.1) {
          s_Arm.wristRollManual(-wristRollSpeed / 13);
          currentTarget.wristRollTarget = s_Arm.getCurrentWristRollPosition();
        }
        else {
          s_Arm.wristRollManual(0);
        }*/
      }
      // if (targetPos == Constants.ArmConstants.grabFromSingleStationPosition) {
      //   if (s_Arm.isClawClosed()) {
      //     clawClosedCnt++;
      //     if (clawClosedCnt > 5) {
      //       this.targetPos = Constants.ArmConstants.restPosition;
      //       clawClosedCnt = 0;
      //     }
      //   }
      // }
    }
    
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.targetPos = originalTarget; 
    //s_Arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
