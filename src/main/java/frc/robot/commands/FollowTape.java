// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ejml.dense.row.decomposition.svd.SafeSvd_DDRM;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveDrive;

public class FollowTape extends CommandBase {
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
  private final ProfiledPIDController xController = new ProfiledPIDController(0.1, 0.001, 0, X_CONSTRAINTS);

 // private static final TrapezoidProfile.Constraints R_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
  //private final ProfiledPIDController rController = new ProfiledPIDController(0.1, 0.001, 0, R_CONSTRAINTS);

  //private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
 // private final ProfiledPIDController yController = new ProfiledPIDController(0.1, 0.1, 0.01, Y_CONSTRAINTS);
  PIDController yController = new PIDController(0.2, 0.01, 0);
  PIDController rController = new PIDController(0.15, 0.0005, 0);
  private SlewRateLimiter rLimiter = new SlewRateLimiter(2);
 SwerveDrive s_Swerve;
 Joystick driverJoystick;
 boolean isDone = false;
 int isInPosCnt = 0;
  /** Creates a new FollowObject. */
  public FollowTape(SwerveDrive s_Swerve, Joystick driverJoystick) {
    this.s_Swerve = s_Swerve;
    this.driverJoystick = driverJoystick;
    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setGoal(0);
    //yController.setGoal(0);
    rController.setSetpoint(180);
    yController.setSetpoint(20.48);
    xController.setTolerance(1);
    yController.setTolerance(0.5);
    s_Swerve.setLimelightPipeline(3);
    isDone = false;
    isInPosCnt = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var robotPose = new Pose2d();
    if (s_Swerve.limelightHasTarget() == 1) {
      var degrees =s_Swerve.getYaw().getDegrees();
      if (degrees < 0) {
        degrees += 360;
      }
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
      Translation2d translation = new Translation2d(ySpeed, xSpeed);
      SmartDashboard.putNumber(
                  "xSpeed",xSpeed);
      SmartDashboard.putNumber(
                    "ySpeed",ySpeed);
      SmartDashboard.putNumber(
                    "rSpeed",rSpeed);
      
      s_Swerve.drive(translation, rSpeed, false, true);
      if (Math.abs(s_Swerve.getLimelightY() - 20.48) < 0.5) {
        isInPosCnt++;
      }
      else {
        isInPosCnt = 0;
      }
      if (isInPosCnt > 20) {
        isDone = true;
      }
    }
    else {
      s_Swerve.stop();
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
