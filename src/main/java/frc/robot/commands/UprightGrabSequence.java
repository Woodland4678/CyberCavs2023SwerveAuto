// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UprightGrabSequence extends SequentialCommandGroup {
  /** Creates a new UprightGrabSequence. */
  public UprightGrabSequence(SwerveDrive s_Swerve, Arm s_Arm) {
    
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Path1M", new PathConstraints(4, 4));
    PPSwerveControllerCommand swerveControllerCommand =
    new PPSwerveControllerCommand(
      examplePath, 
      s_Swerve::getPose, // Pose supplier
      Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
      new PIDController(1,0,0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      new PIDController(1, 0, 0), // Y controller (usually the same values as X controller)
      new PIDController(1, 0,0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      s_Swerve::setModuleStates, // Module states consumer
      false,
      s_Swerve // Requires this drive subsystem
    );
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoGrabUprightCone(s_Arm, s_Swerve));
    var boundingBox = s_Swerve.getBoundingBoxX();
    PathPlannerTrajectory moveToCone;
    double distanceFromFrameToCone = 100.698104 - 2.55144332 * boundingBox[0] + 0.0154124386 * boundingBox[0] * boundingBox[0];
          
    moveToCone = PathPlanner.generatePath(
              new PathConstraints(2, 2), 
              new PathPoint(new Translation2d(0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), s_Swerve.getCurrentVelocity()), // position, heading(direction of travel), holonomic rotation, velocity override
              new PathPoint(new Translation2d(0.5, 0.25), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) // position, heading(direction of travel), holonomic rotation
          );
    addCommands( s_Swerve.followTrajectoryCommand(moveToCone, true));
  }
}
