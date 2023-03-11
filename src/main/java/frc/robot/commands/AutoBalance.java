// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends CommandBase {
  PIDController yController = new PIDController(0.035, 0.005, 0);
  SwerveDrive s_Swerve;
  int count =0;
  int timer = 0;
  int counter = 0;
  double slope = 0;
  boolean isBalanced = false;
  boolean isDone = false;
  double[] rollArray = new double[20];

  /** Creates a new AutoBalance. */
  public AutoBalance(SwerveDrive s_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yController.setSetpoint(-0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if we add a gyro val every 2/50 seconds
    // 20 would be 2/50 * 20 = 0.8 seconds to fill the whole array

    Translation2d translation = new Translation2d(-0.5, 0);
    s_Swerve.drive(translation, 0, false, true);

    if (timer == 2){
      timer =0;
      
      // i=0 is the oldest 
      // i=19 is the latest data
      for (int i = 0; i<18; i++){
        rollArray[i] = rollArray[i+1];
      }

      rollArray[19] = s_Swerve.getGyroRoll();
    


        SmartDashboard.putNumber(
                    "Roll slope", slope);
        SmartDashboard.putNumber(
                      "rollArray[0]", rollArray[0]);
    }
    timer++;

    System.out.println(rollArray[0]);

    if (rollArray[0] != 0) {
      // positive slope, ignore negative slope
      slope = (rollArray[19] - rollArray[0]) / 0.8 ;

      if (slope > 8 ){
        counter ++;

        if (counter > 10){
          counter =0;
          isDone = true;

        }else{
          counter = 0;
        }
      }
    }
    

    /* 
    var ySpeed = yController.calculate(s_Swerve.getGyroRoll());

    if (count > 50){
      count = 0;
      isBalanced = true;
    }
    //Translation2d translation = new Translation2d(-ySpeed, 0);
    //s_Swerve.drive(translation, 0, false, true);
    if (Math.abs(s_Swerve.getGyroRoll()) < 1) {
      count++;
    }
    if (count > 25) {
      isDone = true;
    }
    if (s_Swerve.getGyroRoll() > 3.5) {
      Translation2d translation = new Translation2d(0.5, 0);
      s_Swerve.drive(translation, 0, false, true);
      count = 0;
    }
   else if (s_Swerve.getGyroRoll() < -3.5){
      Translation2d translation = new Translation2d(-0.5, 0);
      s_Swerve.drive(translation, 0, false, true);
      count = 0;
    }
    
    else {
      Translation2d translation = new Translation2d(0, 0);
      s_Swerve.drive(translation, 0, false, true);
      count++;
    }
    */
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
