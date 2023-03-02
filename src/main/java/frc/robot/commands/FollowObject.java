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
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.SwerveDrive;

public class FollowObject extends CommandBase {
  //private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 3);
  //private final ProfiledPIDController xController = new ProfiledPIDController(0.04, 0.001, 0, X_CONSTRAINTS);

 // private static final TrapezoidProfile.Constraints R_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
  //private final ProfiledPIDController rController = new ProfiledPIDController(0.1, 0.001, 0, R_CONSTRAINTS);

  //private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);
 // private final ProfiledPIDController yController = new ProfiledPIDController(0.1, 0.1, 0.01, Y_CONSTRAINTS);
  PIDController yController = new PIDController(0.03, 0, 0);
   PIDController xController = new PIDController(0.1, 0, 0);
  PIDController rController = new PIDController(0.04, 0.0001, 0);
  private SlewRateLimiter rLimiter = new SlewRateLimiter(0.5);
 SwerveDrive s_Swerve;
 boolean isDone = false;
 int isInPosCnt = 0;
 double limelightYTarget = 0;
 double autoDriveXKP = Constants.Swerve.autoDriveXP; 
 double autoDriveXKI = Constants.Swerve.autoDriveXI; 
 double autoDriveXKD = Constants.Swerve.autoDriveXD; 
 double autoDriveXKIz = 0; 
 double autoDriveXKFF = Constants.Swerve.autoDriveXTolerance;  

 double autoDriveYKP = Constants.Swerve.autoDriveYP; 
 double autoDriveYKI = Constants.Swerve.autoDriveYI; 
 double autoDriveYKD = Constants.Swerve.autoDriveYD; 
 double autoDriveYKIz = 0; 
 double autoDriveYKFF = Constants.Swerve.autoDriveYTolerance;  

 double autoDriveRKP = Constants.Swerve.autoDriveRP; 
 double autoDriveRKI = Constants.Swerve.autoDriveRI; 
 double autoDriveRKD = Constants.Swerve.autoDriveRD; 
 double autoDriveRKIz = 0; 
 double autoDriveRKFF = Constants.Swerve.autoDriveRTolerance;  
  /** Creates a new FollowObject. */
  public FollowObject(SwerveDrive s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Auto Drive X P Gain", autoDriveXKP);
    SmartDashboard.putNumber("Auto Drive X I Gain", autoDriveXKI);
    SmartDashboard.putNumber("Auto Drive X D Gain", autoDriveXKD);
    SmartDashboard.putNumber("Auto Drive X I Zone", autoDriveXKIz);
    SmartDashboard.putNumber("Auto Drive X Tolerance", autoDriveXKFF);

    SmartDashboard.putNumber("Auto Drive Y P Gain", autoDriveYKP);
    SmartDashboard.putNumber("Auto Drive Y I Gain", autoDriveYKI);
    SmartDashboard.putNumber("Auto Drive Y D Gain", autoDriveYKD);
    SmartDashboard.putNumber("Auto Drive Y I Zone", autoDriveYKIz);
    SmartDashboard.putNumber("Auto Drive Y Tolerance", autoDriveYKFF);

    SmartDashboard.putNumber("Auto Drive R P Gain", autoDriveRKP);
    SmartDashboard.putNumber("Auto Drive R I Gain", autoDriveRKI);
    SmartDashboard.putNumber("Auto Drive R D Gain", autoDriveRKD);
    SmartDashboard.putNumber("Auto Drive R I Zone", autoDriveRKIz);
    SmartDashboard.putNumber("Auto Drive R Tolerance", autoDriveRKFF);
    isDone = false;
    isInPosCnt = 0;
    //xController.setGoal(0);
    //yController.setGoal(0);
    rController.setSetpoint(s_Swerve.getYaw().getDegrees());
    yController.setSetpoint(119.5);
    xController.setSetpoint(0);
    xController.setTolerance(5);
    yController.setTolerance(3);
    rController.setTolerance(2);
    s_Swerve.setLimelightPipeline(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double autoDriveXP = SmartDashboard.getNumber("Auto Drive X P Gain", 0);
    double autoDriveXI = SmartDashboard.getNumber("Auto Drive X I Gain", 0);
    double autoDriveXD = SmartDashboard.getNumber("Auto Drive X D Gain", 0);
    double autoDriveXIZ = SmartDashboard.getNumber("Auto Drive X I Zone", 0);
    double autoDriveXFF = SmartDashboard.getNumber("Auto Drive X Tolerance", 0);

    double autoDriveYP = SmartDashboard.getNumber("Auto Drive Y P Gain", 0);
    double autoDriveYI = SmartDashboard.getNumber("Auto Drive Y I Gain", 0);
    double autoDriveYD = SmartDashboard.getNumber("Auto Drive Y D Gain", 0);
    double autoDriveYIZ = SmartDashboard.getNumber("Auto Drive Y I Zone", 0);
    double autoDriveYFF = SmartDashboard.getNumber("Auto Drive Y Tolerance", 0);

    double autoDriveRP = SmartDashboard.getNumber("Auto Drive R P Gain", 0);
    double autoDriveRI = SmartDashboard.getNumber("Auto Drive R I Gain", 0);
    double autoDriveRD = SmartDashboard.getNumber("Auto Drive R D Gain", 0);
    double autoDriveRIZ = SmartDashboard.getNumber("Auto Drive R I Zone", 0);
    double autoDriveRFF = SmartDashboard.getNumber("Auto Drive R Tolerance", 0);

    if((autoDriveXP != autoDriveXKP)) {xController.setP(autoDriveXP); }
    if((autoDriveXI != autoDriveXKI)) { xController.setI(autoDriveXI); }
    if((autoDriveXD != autoDriveXKD)) { xController.setD(autoDriveXD); }
    if((autoDriveXIZ != autoDriveXKIz)) { xController.setIntegratorRange(-autoDriveXIZ, autoDriveXIZ); }
    if((autoDriveXFF != autoDriveXKFF)) { xController.setTolerance(autoDriveXFF); } 

    if((autoDriveYP != autoDriveYKP)) {xController.setP(autoDriveYP); }
    if((autoDriveYI != autoDriveYKI)) { xController.setI(autoDriveYI); }
    if((autoDriveYD != autoDriveYKD)) { xController.setD(autoDriveYD); }
    if((autoDriveYIZ != autoDriveYKIz)) { xController.setIntegratorRange(-autoDriveYIZ, autoDriveYIZ); }
    if((autoDriveYFF != autoDriveYKFF)) { xController.setTolerance(autoDriveYFF); } 

    if((autoDriveRP != autoDriveRKP)) {xController.setP(autoDriveRP); }
    if((autoDriveRI != autoDriveRKI)) { xController.setI(autoDriveRI); }
    if((autoDriveRD != autoDriveRKD)) { xController.setD(autoDriveRD); }
    if((autoDriveRIZ != autoDriveRKIz)) { xController.setIntegratorRange(-autoDriveRIZ, autoDriveRIZ); }
    if((autoDriveRFF != autoDriveRKFF)) { xController.setTolerance(autoDriveRFF); } 
    if (RobotContainer.getDriverJoystick().getRawButton(12)) {

    
      var robotPose = new Pose2d();
      var degrees = s_Swerve.getYaw().getDegrees();
      //if (degrees < 0) {
      // degrees += 360;
    // }
    var boundingBoxXY = s_Swerve.getBoundingBoxX();
    var coneAngle = s_Swerve.getConeAngle();
      var rSpeed = rController.calculate(degrees);
      var xSpeed = xController.calculate(s_Swerve.getLimelightX());
      var ySpeed = yController.calculate(s_Swerve.getBoundingBoxX()[0]);
      //var ySpeed = yController.calculate(s_Swerve.getLimelightObjectSize());
      //var ySpeed = yController.calculate(s_Swerve.getLimelightObjectSize());
      // if (Math.abs(rSpeed) < 0.7) {
      //   xSpeed = rLimiter.calculate(rController.calculate(s_Swerve.getYaw().getDegrees()));
      // }
      // else {
      //   ySpeed = ySpeed * 0.5;
      //   xSpeed = 0;
      // }
      Translation2d translation = new Translation2d(ySpeed, xSpeed);
      // SmartDashboard.putNumber(
      //             "xSpeed",xSpeed);
      // SmartDashboard.putNumber(
      //               "ySpeed",ySpeed);
      // SmartDashboard.putNumber(
      //               "rSpeed",0);
      //SmartDashboard.putNumber(
      //              "coneAngle",coneAngle);
      
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
    return false;
  }
}
