// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CalibrateArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveBack extends SequentialCommandGroup {
  /** Creates a new MoveBack. */
  public MoveBack(SwerveDrive s_Swerve, Arm s_Arm) {
    PathPlannerTrajectory move = PathPlanner.loadPath("Move Out Of Zone", new PathConstraints(2, 2));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> s_Swerve.zeroGyro()),
      new InstantCommand(() -> s_Swerve.resetSwerveModuleAngles()),
      new ParallelCommandGroup(s_Swerve.followTrajectoryCommand(move, true), new CalibrateArm(s_Arm))
    );
  }
}
