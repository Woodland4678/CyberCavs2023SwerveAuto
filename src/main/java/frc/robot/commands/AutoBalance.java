// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends CommandBase {
  SwerveDrive s_Swerve;
  int count =0;
  boolean isBalanced = false;
  /** Creates a new AutoBalance. */
  public AutoBalance(SwerveDrive s_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (count > 50){
      count = 0;
      isBalanced = true;
    }

    if (s_Swerve.getGyroRoll() > 1) {
      Translation2d translation = new Translation2d(0.5, 0);
      s_Swerve.drive(translation, 0, false, true);
      count = 0;
    }
   else if (s_Swerve.getGyroRoll() < -1.5){
      Translation2d translation = new Translation2d(-0.5, 0);
      s_Swerve.drive(translation, 0, false, true);
      count = 0;
    }
    else {
      Translation2d translation = new Translation2d(0, 0);
      s_Swerve.drive(translation, 0, false, true);
      count++;
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
    return isBalanced;
  }
}
