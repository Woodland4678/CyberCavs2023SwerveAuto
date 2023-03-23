// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CalibrateArm;
import frc.robot.commands.YeetCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOnly extends SequentialCommandGroup {
  /** Creates a new ScoreOnly. */
  public ScoreOnly(Arm s_Arm, SwerveDrive s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(new InstantCommand(() -> s_Swerve.zeroGyro()), new InstantCommand(() -> s_Swerve.resetSwerveModuleAngles()), new InstantCommand(() -> s_Swerve.limelightDown()), new InstantCommand(() -> s_Swerve.setLimelightPipeline(6)), new YeetCube(s_Arm)),
      new CalibrateArm(s_Arm)
    );
  }
}
