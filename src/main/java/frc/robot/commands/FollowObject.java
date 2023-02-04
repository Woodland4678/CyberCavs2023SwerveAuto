// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveDrive;

public class FollowObject extends CommandBase {
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 3);
  private final ProfiledPIDController xController = new ProfiledPIDController(0.1, 0.001, 0, X_CONSTRAINTS);

 // private static final TrapezoidProfile.Constraints R_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
  //private final ProfiledPIDController rController = new ProfiledPIDController(0.1, 0.001, 0, R_CONSTRAINTS);

  //private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
 // private final ProfiledPIDController yController = new ProfiledPIDController(0.1, 0.1, 0.01, Y_CONSTRAINTS);
  PIDController yController = new PIDController(0.5, 0.001, 0);
  PIDController rController = new PIDController(0.1, 0.0001, 0);
  private SlewRateLimiter rLimiter = new SlewRateLimiter(0.5);
 SwerveDrive s_Swerve;
 boolean isDone = false;
 int isInPosCnt = 0;
  /** Creates a new FollowObject. */
  public FollowObject(SwerveDrive s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    isInPosCnt = 0;
    xController.setGoal(0);
    //yController.setGoal(0);
    rController.setSetpoint(0);
    yController.setSetpoint(0);
    xController.setTolerance(1);
    yController.setTolerance(1);
    s_Swerve.setLimelightPipeline(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var robotPose = new Pose2d();
    var degrees =s_Swerve.getYaw().getDegrees();
    //if (degrees < 0) {
     // degrees += 360;
   // }
    var rSpeed = rController.calculate(degrees);
    var xSpeed = xController.calculate(s_Swerve.getLimelightX());
    var ySpeed = yController.calculate(s_Swerve.getLimelightY());
    // if (Math.abs(rSpeed) < 0.7) {
    //   xSpeed = rLimiter.calculate(rController.calculate(s_Swerve.getYaw().getDegrees()));
    // }
    // else {
    //   ySpeed = ySpeed * 0.5;
    //   xSpeed = 0;
    // }
    Translation2d translation = new Translation2d(-ySpeed, xSpeed);
    SmartDashboard.putNumber(
                "xSpeed",xSpeed);
    SmartDashboard.putNumber(
                  "ySpeed",ySpeed);
    // SmartDashboard.putNumber(
    //               "rSpeed",0);
    
    s_Swerve.drive(translation, rSpeed, false, true);
    if (s_Swerve.getLimelightY() < 0.2) {
      isInPosCnt++;
    }
    else {
      isInPosCnt = 0;
    }
    if (isInPosCnt > 15) {
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
