// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;

import com.ctre.phoenixpro.signals.System_StateValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends CommandBase {
  SwerveDrive s_Swerve;
  double ySpeed = 0;
  boolean isDone = false;
  int moveXDirectionCnt = 0;
  int isBalancedCnt = 0;
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
    isBalancedCnt = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroPitchValue = s_Swerve.getGyroPitch();
    if (Math.abs(gyroPitchValue) > Constants.Swerve.AutobalanceRollTolerance && DriverStation.getMatchTime() > 0.11) {
      isBalancedCnt = 0;
      if (gyroPitchValue < 0) {        
        ySpeed = -0.45;
        
      }
      else {
        ySpeed = 0.6;
      }
      Translation2d translation = new Translation2d(ySpeed, 0);
      s_Swerve.drive(translation, 0, false, true);
    }
    else {      
      isBalancedCnt++;
      s_Swerve.stop();
      if (isBalancedCnt > 60 || DriverStation.getMatchTime() < 0.1) {
        s_Swerve.setToXOrientation();      
        isDone = true;      
      }
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
