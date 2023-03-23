// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.lang.invoke.ConstantBootstraps;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoGrabUprightCone;
import frc.robot.commands.AutoScoreHigh;
import frc.robot.commands.CalibrateArm;
import frc.robot.commands.FollowObject;
import frc.robot.commands.MoveArm;
import frc.robot.commands.YeetCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BumpThreeGamePiece extends SequentialCommandGroup {
  /** Creates a new TwoGamePieceAndBalance. */
  public BumpThreeGamePiece(SwerveDrive s_Swerve, Arm s_Arm,  CommandXboxController operatorJoystick, PathPlannerTrajectory[] paths) {
    // PathPlannerTrajectory goTo2ndGamePiece = PathPlanner.loadPath("Bump Auto Path 1", new PathConstraints(2, 2));
    // PathPlannerTrajectory bring2ndGamePieceBack = PathPlanner.loadPath("Bump Auto Path 2", new PathConstraints(2, 2));
    // PathPlannerTrajectory goTo3rdGamePiece = PathPlanner.loadPath("Bump Go To 3rd Piece", new PathConstraints(2, 2));
    // PathPlannerTrajectory bring3rdGamePieceBack = PathPlanner.loadPath("Bump Bring 3rd Piece Back", new PathConstraints(2,2));
    
    double thirdGamePieceGrabAngle = -45;
    if (DriverStation.getAlliance() == Alliance.Blue) {
      thirdGamePieceGrabAngle = 45;
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> s_Swerve.zeroGyro()), 
         new InstantCommand(() -> s_Swerve.resetSwerveModuleAngles()),
         new YeetCube(s_Arm),         
         new ParallelCommandGroup(s_Swerve.followTrajectoryCommand(PathPlannerTrajectory.transformTrajectoryForAlliance(paths[0], DriverStation.getAlliance()), true), new CalibrateArm(s_Arm), new InstantCommand(() -> s_Swerve.setHeadlights(true)), new InstantCommand(() -> s_Swerve.limelightDown())),
         new AutoGrabUprightCone(s_Arm, s_Swerve, 0, false),
         s_Swerve.followTrajectoryCommand(PathPlannerTrajectory.transformTrajectoryForAlliance(paths[1], DriverStation.getAlliance()), true),
         new AutoScoreHigh(s_Arm, s_Swerve, true, operatorJoystick, true, 10),
         new InstantCommand(() -> s_Arm.openClaw()),
         new ParallelCommandGroup(s_Swerve.followTrajectoryCommand(PathPlannerTrajectory.transformTrajectoryForAlliance(paths[2], DriverStation.getAlliance()), true), new MoveArm(s_Arm, Constants.ArmConstants.restPositionAuto, null)),
         new AutoGrabUprightCone(s_Arm, s_Swerve, thirdGamePieceGrabAngle, false),
         s_Swerve.followTrajectoryCommand(PathPlannerTrajectory.transformTrajectoryForAlliance(paths[3], DriverStation.getAlliance()), true)
        //  new AutoScoreHigh(s_Arm, s_Swerve, true, operatorJoystick, true, 20),
        //  new InstantCommand(() -> s_Arm.openClaw()),
        //  new MoveArm(s_Arm, Constants.ArmConstants.restPositionAuto, null)
        //  s_Swerve.followTrajectoryCommand(bring2ndGamePieceBack, true),
        //  new AutoScoreHigh(s_Arm, s_Swerve, true, 0.0),
        //  s_Swerve.followTrajectoryCommand(goAutoBalance, true)
         );
  }
}
