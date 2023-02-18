// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmMoveTrapezoidal extends CommandBase {
  Arm s_Arm;
  int posNum = 1;
  /** Creates a new ArmMoveTrapezoidal. */
  public ArmMoveTrapezoidal(Arm s_Arm, int pos) {
    this.s_Arm = s_Arm;
    addRequirements(s_Arm);
    this.posNum = pos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (posNum == 1) {
      s_Arm.moveArmtrapezoidalInit(s_Arm.getCurrentElbowPosition(), 
                                    s_Arm.getCurrentShoulderPosition(), 
                                    Constants.ArmConstants.position1X, 
                                    Constants.ArmConstants.position1Y);
    }
    else {
      s_Arm.moveArmtrapezoidalInit(s_Arm.getCurrentElbowPosition(), 
                                    s_Arm.getCurrentShoulderPosition(),
                                    Constants.ArmConstants.position2X, 
                                    Constants.ArmConstants.position2Y);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Arm.moveArmtrapezoidal();
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
