// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

public class orientationTest extends CommandBase {
  SwerveDrive s_Swerve;
  double angle = 0;
  Arm s_Arm;
  ArmPosition currentTarget = Constants.ArmConstants.pickupToRestIntermediatePosition;
  double currentArmError;
  PIDController xController = new PIDController(Constants.Swerve.autoDriveConePickupXP, Constants.Swerve.autoDriveConePickupXI, Constants.Swerve.autoDriveConePickupXD);
  PIDController yController = new PIDController(0.05,0,0);
  PIDController rController = new PIDController(Constants.Swerve.autoDriveConePickupRP, Constants.Swerve.autoDriveConePickupRI, Constants.Swerve.autoDriveConePickupRD);

  /** Creates a new orientationTest. */
  public orientationTest(SwerveDrive s_Swerve, Arm s_Arm) {
    this.s_Swerve = s_Swerve;
    this.s_Arm = s_Arm;
    currentArmError = 0;
    addRequirements(s_Swerve, s_Arm);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //s_Swerve.setHeadlights(true);
    s_Swerve.limelightDown();
    s_Swerve.setLimelightPipeline(1);
    s_Swerve.setLimelightLED(false);
    yController.reset();
    yController.setSetpoint(15);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentArmError = s_Arm.MoveArm(currentTarget);
    if (currentArmError < 4) {
      currentTarget = Constants.ArmConstants.pickupPosition;      
    }
    
    // angle += 0.25;
    // if (angle > 180) {
    //   angle = -180;
    // }
    var boundingBox = s_Swerve.getBoundingBoxMinMaxX();
    double value = SwerveDrive.GetConePosition(boundingBox[0], boundingBox[1], s_Swerve.getConeAngle()).GetY();
    SmartDashboard.putNumber("Wilms distance to cone", value);
    SmartDashboard.putNumber("Wilms set cone angle", s_Swerve.getConeAngle());
    double ySpeed = yController.calculate(value);
    if (yController.atSetpoint()) {
      s_Swerve.stop();
    }
    else {
      Translation2d translation = new Translation2d(-ySpeed, 0);
      //s_Swerve.drive(translation, 0, false,true);
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
