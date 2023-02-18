// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmMoveSmartMotion extends CommandBase {
  Arm s_Arm;
  int pos = 1;
  double targetX;
  double targetY;
  /** Creates a new ArmMoveSmartMotion. */
  public ArmMoveSmartMotion(Arm s_Arm, int pos) {
    this.s_Arm = s_Arm;
    this.pos = pos;
    addRequirements(s_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Arm.resetSmartMotionState();
    if (pos == 1) {
      targetX = Constants.ArmConstants.position1X;
      targetY = Constants.ArmConstants.position1Y;
    }
    else {
      targetX = Constants.ArmConstants.position2X;
      targetY = Constants.ArmConstants.position2Y;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Arm.MoveArmSmartMotion(targetX, targetY, 1, 1, 1, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
