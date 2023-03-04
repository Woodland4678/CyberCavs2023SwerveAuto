// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
  Arm s_Arm;
  ArmPosition targetPos;
  double currentArmError = 0;
  boolean doneIntermediateMovement = false;
  ArmPosition currentTarget;
  Joystick operatorJoystick;
  boolean armOkayToMove = true;
  /** Creates a new MoveArm. */
  public MoveArm(Arm s_Arm, ArmPosition targetPos, Joystick operatorJoystick) {
    this.s_Arm = s_Arm;
    this.targetPos = targetPos;
    this.operatorJoystick = operatorJoystick;
    addRequirements(s_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armOkayToMove = true;
    //if we're going to pickup position set the target as the intermediate position first
    if (this.targetPos == Constants.ArmConstants.pickupPosition) {
     currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition; 
    }
    else if (this.targetPos == Constants.ArmConstants.restPosition) {
      currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
    }
    else if (this.targetPos == Constants.ArmConstants.scoreConeHighPosition && s_Arm.getGamePieceMode() == Constants.ArmConstants.cubeMode) {
      currentTarget = Constants.ArmConstants.scoreCubeHighPosition;
    }
    else if (this.targetPos == Constants.ArmConstants.scoreConeMediumPosition && s_Arm.getGamePieceMode() == Constants.ArmConstants.cubeMode) {
      currentTarget = Constants.ArmConstants.scoreCubeMediumPosition;
    }
    else if (this.targetPos == Constants.ArmConstants.scoreConeHighPosition) {
      currentTarget = Constants.ArmConstants.restToScoreIntermediatePosition;
    }
    else {
      currentTarget = targetPos;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (currentTarget == Constants.ArmConstants.grabConePosition || currentTarget == Constants.ArmConstants.grabCubePosition || currentTarget == Constants.ArmConstants.grabUprightConePosition) {
      if (s_Arm.getCurrentXPosition() < 15) {
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
      if (Math.abs(currentArmError) < 5) { //handles intermediate positions
        currentTarget = targetPos;
        double wristRollSpeed = operatorJoystick.getRawAxis(0);
        if (Math.abs(wristRollSpeed) > 0.1) {
          s_Arm.wristRollManual(-wristRollSpeed / 13);
          currentTarget.wristRollTarget = s_Arm.getCurrentWristRollPosition();
        }
        else {
          s_Arm.wristRollManual(0);
        }
      }
    }
    
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //s_Arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
