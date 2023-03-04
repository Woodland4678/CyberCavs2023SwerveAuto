// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class GrabGamePiece extends CommandBase {
  Arm s_Arm;
  /** Creates a new GrabGamePiece. */
  public GrabGamePiece(Arm s_Arm) {
    this.s_Arm = s_Arm;
    addRequirements(s_Arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.ArmConstants.grabConePosition.wristRollTarget = s_Arm.getCurrentWristRollPosition();
    Constants.ArmConstants.grabCubePosition.wristRollTarget = s_Arm.getCurrentWristRollPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Arm.getCurrentXPosition() > 15) {
      var currentArmError = s_Arm.MoveArm(Constants.ArmConstants.grabConePosition);
      if (Math.abs(currentArmError) < 3) {
        s_Arm.closeClaw();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
