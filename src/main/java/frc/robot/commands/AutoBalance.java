// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenixpro.signals.System_StateValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends CommandBase {
  SwerveDrive s_Swerve;
  double ySpeed = 0;
  boolean isDone = false;
  int moveXDirectionCnt = 0;
  /** Creates a new AutoBalance. */
  public AutoBalance(SwerveDrive s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moveXDirectionCnt = 0;
    isDone = false;
    ySpeed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroRollValue = s_Swerve.getGyroRoll();
    if (Math.abs(gyroRollValue) > Constants.Swerve.AutobalanceRollTolerance) {
      if (gyroRollValue < 0) {        
        ySpeed = -0.4;
        
      }
      else {
        ySpeed = 0.4;
      }
      Translation2d translation = new Translation2d(ySpeed, 0);
      s_Swerve.drive(translation, 0, false, true);
    }
    else {      
      if (moveXDirectionCnt < 25) {
        Translation2d translation = new Translation2d(0, 0.1);
        s_Swerve.drive(translation, 0, false, true);
      }
      moveXDirectionCnt++;
      isDone = true;      
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
    return isDone;
  }
}
