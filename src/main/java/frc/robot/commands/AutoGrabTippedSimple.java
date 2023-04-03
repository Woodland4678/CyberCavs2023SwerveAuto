// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class AutoGrabTippedSimple extends CommandBase {
  Arm s_Arm;
  SwerveDrive s_Swerve;
  PIDController xController = new PIDController(0.09, Constants.Swerve.autoDriveConePickupXI, Constants.Swerve.autoDriveConePickupXD);
  PIDController yController = new PIDController(0.09, Constants.Swerve.autoDriveConePickupYI, Constants.Swerve.autoDriveConePickupYD);
  PIDController rController = new PIDController(Constants.Swerve.autoDriveConePickupRP, Constants.Swerve.autoDriveConePickupRI, Constants.Swerve.autoDriveConePickupRD);
  ArmPosition currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
  double currentArmError = 0;
  double xSpeed = 0;
  double ySpeed = 0;
  double rSpeed = 0;
  double degrees = 0;
  int state = 0;
  int grabConeCnt = 0;
  boolean isDone = false;
  /** Creates a new AutoGrabTippedSimple. */
  public AutoGrabTippedSimple(SwerveDrive s_Swerve, Arm s_Arm) {    
    this.s_Swerve = s_Swerve;
    this.s_Arm = s_Arm;
    addRequirements(s_Swerve, s_Arm);
    currentArmError = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
    xSpeed = 0;
    ySpeed = 0;
    rSpeed = 0;
    xController.reset();
    yController.reset();
    rController.reset();    
    s_Swerve.setLimelightPipeline(6);
    s_Swerve.setLimelightLED(false);
    xController.setSetpoint(Constants.Swerve.autoGrabTippedBaseTowardsXTarget);
    yController.setSetpoint(Constants.Swerve.autoGrabTippedBaseTowardsYTarget);
    xController.setTolerance(1);
    yController.setTolerance(1);
    degrees = s_Swerve.getYaw().getDegrees();
    rController.setSetpoint(s_Swerve.getYaw().getDegrees());
    grabConeCnt = 0;
    //s_Swerve.setHeadlights(true);
    s_Swerve.limelightDown();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentArmError = s_Arm.MoveArm(currentTarget);
    switch(state) {
      case 0:
        
        if (currentArmError < 4) {
          currentTarget = Constants.ArmConstants.pickupPosition;      
        }
        var 
        rSpeed = rController.calculate(degrees);
        xSpeed = xController.calculate(s_Swerve.getLimelightX()); 
        ySpeed = yController.calculate(s_Swerve.getLimelightY()); 
        if (s_Swerve.getLimelightY() < yController.getSetpoint()) {
          ySpeed = 0;
        }
        
        Translation2d translation = new Translation2d(-ySpeed, xSpeed);
        
        
        s_Swerve.drive(translation, rSpeed, false, true);
        if (xController.atSetpoint() && yController.atSetpoint()) {
          state++;
          s_Swerve.stop();
        }
      break;
      case 1:
        currentTarget = Constants.ArmConstants.grabConePosition;
        state++;
      break;
      case 2:
        if (currentArmError < 5) {
          grabConeCnt++;
        }
        if (grabConeCnt > 10) {
          state++;
          grabConeCnt = 0;
        }
      break;
      case 3:
        s_Arm.closeClaw();
        grabConeCnt++;
        if (grabConeCnt > 25) {
          state++;
        }
      break;
      case 4:
        currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
        state++;
      break;
      case 5:
        isDone = true;
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
