// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class YeetCube extends CommandBase {
  Arm s_Arm;
  int doneCnt = 0;
  boolean isDone;
  /** Creates a new YeetCube. */
  public YeetCube(Arm s_Arm) {
    this.s_Arm = s_Arm;
    addRequirements(s_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    doneCnt = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var currentArmError = s_Arm.MoveArm(Constants.ArmConstants.yeetCubePosition);
    if (currentArmError < 42) {
      s_Arm.openClaw();
      doneCnt++;
    }
    if (doneCnt > 2) {
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (DriverStation.isAutonomous()) {
      s_Arm.MoveArm(Constants.ArmConstants.pickupToRestIntermediatePositionAuto);
    }
    else {
      s_Arm.MoveArm(Constants.ArmConstants.pickupToRestIntermediatePosition);
    }
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
