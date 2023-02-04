// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.FollowObject;
import frc.robot.commands.FollowTape;
import frc.robot.subsystems.SwerveDrive;

public class BaseAuto extends SequentialCommandGroup {
  public BaseAuto(SwerveDrive s_Swerve) {
    PathPlannerTrajectory examplePath = PathPlanner.loadPath("Test Path 3", new PathConstraints(4, 4));
    PathPlannerTrajectory path2 = PathPlanner.loadPath("Test Path 4", new PathConstraints(4, 4));
    PathPlannerTrajectory path3 = PathPlanner.loadPath("Test Path 5", new PathConstraints(4, 4));
    PathPlannerTrajectory path4 = PathPlanner.loadPath("Test Path 6", new PathConstraints(4, 4));
    /*  TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics);
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);*/

    var thetaController =
    new ProfiledPIDController(
      Constants.AutoConstants.kPThetaController,
        1,
        0.1,
        Constants.AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);

/*SwerveControllerCommand swerveControllerCommand =
    new SwerveControllerCommand(
      examplePath,
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 1, 0.1),
        new PIDController(Constants.AutoConstants.kPYController, 1, 0.1),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);*/
  PPSwerveControllerCommand swerveControllerCommand =
    new PPSwerveControllerCommand(
      examplePath, 
      s_Swerve::getPose, // Pose supplier
      Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
      new PIDController(5, 1, 0.7), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      new PIDController(5, 1, 0.7), // Y controller (usually the same values as X controller)
      new PIDController(4, 1, 0.7), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      s_Swerve::setModuleStates, // Module states consumer
      false,
      s_Swerve // Requires this drive subsystem
);
PPSwerveControllerCommand swerveControllerCommand2 =
new PPSwerveControllerCommand(
  path2, 
  s_Swerve::getPose, // Pose supplier
  Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
  new PIDController(5, 1, 0.7), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
  new PIDController(5, 1, 0.7), // Y controller (usually the same values as X controller)
  new PIDController(4, 1, 0.7), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
  s_Swerve::setModuleStates, // Module states consumer
  false,
  s_Swerve // Requires this drive subsystem
);
PPSwerveControllerCommand swerveControllerCommand3 =
new PPSwerveControllerCommand(
  path3, 
  s_Swerve::getPose, // Pose supplier
  Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
  new PIDController(5, 1, 0.7), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
  new PIDController(5, 1, 0.7), // Y controller (usually the same values as X controller)
  new PIDController(4, 1, 0.7), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
  s_Swerve::setModuleStates, // Module states consumer
  false,
  s_Swerve // Requires this drive subsystem
);
PPSwerveControllerCommand swerveControllerCommand4 =
new PPSwerveControllerCommand(
  path4, 
  s_Swerve::getPose, // Pose supplier
  Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
  new PIDController(5, 1, 0.7), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
  new PIDController(5, 1, 0.7), // Y controller (usually the same values as X controller)
  new PIDController(4, 1, 0.7), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
  s_Swerve::setModuleStates, // Module states consumer
  false,
  s_Swerve // Requires this drive subsystem
);


//addCommands(
     // s_Swerve.followTrajectoryCommand(examplePath, true));
     addCommands(
         new InstantCommand(() -> s_Swerve.resetOdometry(examplePath.getInitialHolonomicPose())),
         swerveControllerCommand,
         new FollowObject(s_Swerve),
         swerveControllerCommand2,
         new FollowTape(s_Swerve),
         swerveControllerCommand3,
         new FollowObject(s_Swerve),
         swerveControllerCommand4,
         new FollowTape(s_Swerve)
         );
  }
}
