// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.LEDModes;
import frc.robot.autos.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final CommandXboxController driver = new CommandXboxController(0);
  private static final CommandXboxController operator = new CommandXboxController(1);
  //private static final Joystick operator = new Joystick(1);
  private static final Joystick autoBox = new Joystick(2);

    PathPlannerTrajectory goTo2ndGamePiece = PathPlanner.loadPath("Non Bump Path 1", new PathConstraints(4.1, 4));
    PathPlannerTrajectory bring2ndGamePieceBack = PathPlanner.loadPath("Non Bump Path 2", new PathConstraints(4.1, 4.5));
    PathPlannerTrajectory goAutoBalance = PathPlanner.loadPath("Non Bump Path Auto Balance After 2", new PathConstraints(3, 2));
  PathPlannerTrajectory[] nonBump2GamePieceAndBalancePaths = {
    goTo2ndGamePiece,
    bring2ndGamePieceBack,
    goAutoBalance,
  };
   
    PathPlannerTrajectory goTo3rdGamePiece = PathPlanner.loadPath("Non Bump Path Game Piece 3", new PathConstraints(4.2, 4.5));
    PathPlannerTrajectory bring3rdGamePieceBack = PathPlanner.loadPath("Non Bump Path Move 3rd Piece", new PathConstraints(4.3, 4.5));
  
  PathPlannerTrajectory[] nonBump3GamePiece = {
    goTo2ndGamePiece,
    bring2ndGamePieceBack,
    goTo3rdGamePiece,
    bring3rdGamePieceBack
  };

    PathPlannerTrajectory goBalanceAfter3rdGrabbed = PathPlanner.loadPath("Non Bump 2.5 Game Piece Balance", new PathConstraints(2.2, 3));
  PathPlannerTrajectory[] nonBump2AndAHalfGamePieceAndBalance = {
    goTo2ndGamePiece,
    bring2ndGamePieceBack,
    goTo3rdGamePiece,
    goBalanceAfter3rdGrabbed
  };


    PathPlannerTrajectory bumpGoTo2ndGamePiece = PathPlanner.loadPath("Bump Auto Path 1", new PathConstraints(3, 4.1));
    PathPlannerTrajectory bumpBring2ndGamePieceBack = PathPlanner.loadPath("Bump Auto Path 2", new PathConstraints(3, 3));
    PathPlannerTrajectory bumpGoAutoBalance = PathPlanner.loadPath("Bump Auto Balance After 2", new PathConstraints(2.5, 2));
    PathPlannerTrajectory[] bumpTwoGamePieceAndBalancePaths = {
      bumpGoTo2ndGamePiece,
      bumpBring2ndGamePieceBack,
      bumpGoAutoBalance
    };

    PathPlannerTrajectory bumpGoTo3rdGamePiece = PathPlanner.loadPath("Bump Go To 3rd Piece", new PathConstraints(3, 4.1));
    PathPlannerTrajectory bumpBring3rdGamePieceBack = PathPlanner.loadPath("Bump Bring 3rd Piece Back", new PathConstraints(2,3));
    PathPlannerTrajectory[] bumpThreeGamePiecePaths = {
      bumpGoTo2ndGamePiece,
      bumpBring2ndGamePieceBack,
      bumpGoTo3rdGamePiece,
      bumpBring3rdGamePieceBack
    };
    /* Drive Controls */
  //private final int translationAxis = driver.left;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

 
  
  

  /* Operator Buttons */
  // private final JoystickButton operatorBtnY =
  //     new JoystickButton(operator, 4);
  // private final JoystickButton operatorBtnX =
  //     new JoystickButton(operator, 1);
  // private final JoystickButton operatorBtnA =
  //     new JoystickButton(operator, 2);
  // private final JoystickButton operatorBtnB =
  //     new JoystickButton(operator, 3);
  // private final JoystickButton operatorBtnLB =
  //     new JoystickButton(operator, 5);
  // private final JoystickButton operatorBtnRB =
  //     new JoystickButton(operator, 6);
  // private final JoystickButton operatorBtnBack =
  //     new JoystickButton(operator, 9);
  // private final JoystickButton operatorBtnStart =
  //     new JoystickButton(operator, 10);
  // private final JoystickButton operatorBtnRT =
  //     new JoystickButton(operator, 8);
  // private final JoystickButton operatorBtnLT =
  //     new JoystickButton(operator, 7); 
  // private final JoystickButton operatorPOV90 =
  //     new JoystickButton(operator, operator.getPOV(1)); //doesn't work
  // private final JoystickButton operatorBtnRightStick =
  //     new JoystickButton(operator, 12); 
  // private final JoystickButton operatorBtnLeftStick =
  //     new JoystickButton(operator, 11); 


  /* Subsystems */
  private final SwerveDrive s_Swerve = new SwerveDrive();
  private final Arm s_Arm = new Arm();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            false,
            driver.rightTrigger(),
            driver.rightBumper()));



    // Configure the button bindings
    configureButtonBindings();
  }
  private void configureButtonBindings() {
    /* Driver Buttons */
    driver.back().onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
   // driverBtnA.whileTrue(new UprightGrabSequence(s_Swerve, s_Arm));
    driver.x().onTrue(new InstantCommand(() -> s_Arm.openClaw()));
    driver.b().onTrue(new InstantCommand(() -> s_Arm.closeClaw()));
    //driverBtnRB.whileTrue(new YeetCube(s_Arm));
    driver.y().whileTrue(new AutoScoreHigh(s_Arm, s_Swerve, true, operator, true, 20)); //score high
    driver.a().whileTrue(new AutoScoreHigh(s_Arm, s_Swerve, false,  operator, true, 20)); //score medium
    driver.start().onTrue(new InstantCommand(() -> s_Swerve.resetSwerveModuleAngles()));
    //driver.rightBumper().whileTrue(new AutoGrabUprightCone(s_Arm, s_Swerve));
    driver.leftBumper().whileTrue(new AutoGrabCube(s_Swerve, s_Arm));
    driver.leftStick().whileTrue(new YeetCube(s_Arm));
    driver.rightStick().onTrue(new InstantCommand(() -> s_Swerve.setToXOrientation()));
    //driver.rightTrigger().whileTrue(new YeetCube(s_Arm));
    //driver.rightTrigger().whileTrue(new AutoBalance(s_Swerve));
    driver.pov(180).whileTrue(new orientationTest(s_Swerve, s_Arm));
    driver.leftTrigger().whileTrue(new AutoGrabTippedSimple(s_Swerve, s_Arm));
    //driver.pov(180).onTrue(new InstantCommand(() -> s_Swerve.setToXOrientation()));
    driver.pov(0).whileTrue(new AutoGrabUprightCone(s_Arm, s_Swerve, 0, false));
    
    //driver.leftTrigger(0.5).whileTrue(new YeetCube(s_Arm));
    //followTape.whileTrue(new FollowTape(s_Swerve, driver));
    //autoBalance.whileTrue(new AutoBalance(s_Swerve));
    //moveArmPos1.whileTrue(new OpenClaw(s_Arm, 1));
    //moveArmPos2.whileTrue(new OpenClaw(s_Arm, 2));
    //moveArmPos1.whileTrue(new ArmMovePIDOnly(s_Arm, 1));
    //moveArmPos2.whileTrue(new ArmMovePIDOnly(s_Arm, 2));

    /* Operator Buttons */
    operator.y().onTrue(new MoveArm(s_Arm,Constants.ArmConstants.scoreConeHighPosition, operator));
    operator.b().onTrue(new MoveArm(s_Arm,Constants.ArmConstants.scoreConeMediumPosition, operator));
    operator.a().onTrue(new MoveArm(s_Arm,Constants.ArmConstants.scoreLowPosition, operator));
    operator.x().onTrue(new MoveArm(s_Arm,Constants.ArmConstants.restPosition, operator));
    operator.leftBumper().onTrue(new InstantCommand(() -> s_Arm.coneMode()));
    operator.rightBumper().onTrue(new InstantCommand(() -> s_Arm.cubeMode()));
    //operatorBtnRB.onTrue(new GrabGamePiece(s_Arm));
    operator.back().onTrue(new CalibrateArm(s_Arm));
    operator.rightTrigger().onTrue(new MoveArm(s_Arm,Constants.ArmConstants.grabUprightConePosition, operator));
    operator.leftTrigger().onTrue(new MoveArm(s_Arm,Constants.ArmConstants.grabFromSingleStationPosition, operator));
    operator.pov(90).onTrue(new MoveArm(s_Arm, Constants.ArmConstants.pickupPosition, operator));
    operator.pov(180).whileTrue(new AutoBalance(s_Swerve));
    operator.pov(270).onTrue(new InstantCommand(() -> s_Arm.resetToAbsoluteEncoder()));
    //operatorBtnRightStick.onTrue(new MoveArm(s_Arm, Constants.ArmConstants.headTiltForVideoPosition, operator));
    //operatorBtnLeftStick.onTrue(new tempHeadTilt(s_Arm));
  }


  public static CommandXboxController getOperatorJoystick() {
    return operator;
  }
  public static CommandXboxController getDriverJoystick() {
    return driver;
  }
  public void resetArmAngles() {
    s_Arm.resetToAbsoluteEncoder();
  }  
  public void moveShoulder(double speed) {
    s_Arm.runShoulderMotor(speed);
  }
  public void moveElbow(double speed) {
    s_Arm.runElbowMotor(speed);
  }
  public void setElbowPIDF(double p, double i, double iZone, double d, double f) {
    s_Arm.setElbowPIDF(p, i, iZone, d, f);
  }
  public void setShoulderPIDF(double p, double i, double iZone, double d, double f) {
    s_Arm.setShoulderPIDF(p, i, iZone, d, f);
  }
  public void setWristPitchPIDF(double p, double i, double iZone, double d, double f) {
    s_Arm.setWristPitchPIDF(p, i, iZone, d, f);
  }
  public void setWristRollPIDF(double p, double i, double iZone, double d, double f) {
    s_Arm.setWristRollPIDF(p, i, iZone, d, f);
  }
  public boolean isArmStartOkay() {
    return s_Arm.isArmStartOkay();
  }
  public void setArmMayMove(boolean mayMove) {
    s_Arm.setArmCanMove(mayMove);
  }
  public void setLEDsToDisabledMode() {
    s_Arm.setLEDMode(LEDModes.ROBOTDISABLEDPATTERN);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    int autoBoxSwitch1 = 0;
    int autoBoxSwitch2 = 0;
    if (autoBox.getRawButton(1)) {
      autoBoxSwitch1 += 1;
    }
    if (autoBox.getRawButton(2)) {
      autoBoxSwitch1 += 2;
    }
    if (autoBox.getRawButton(3)) {
      autoBoxSwitch1 += 4;
    }
    if (autoBoxSwitch1 == 0) {
      autoBoxSwitch1 = 6;
    }
    else if (autoBoxSwitch1 > 5) {
      autoBoxSwitch1 = autoBoxSwitch1 + 1;
    }
    

    if (autoBox.getRawButton(8)) {
      autoBoxSwitch2 += 1;
    }
    if (autoBox.getRawButton(9)) {
      autoBoxSwitch2 += 2;
    }
    if (autoBox.getRawButton(10)) {
      autoBoxSwitch2 += 4;
    }
    if (autoBoxSwitch2 == 0) {
      autoBoxSwitch2 = 6;
    }
    else if (autoBoxSwitch2 > 5) {
      autoBoxSwitch2 = autoBoxSwitch2 + 1;
    }
    if (autoBoxSwitch2 == 1) {
      if (autoBoxSwitch1 == 1) {
        return new MiddleScoreAndAutoBalance(s_Swerve, s_Arm);
      }
      else if (autoBoxSwitch1 == 2) {
        return new NonBump2AndAHalfAndBalance(s_Swerve, s_Arm, operator, nonBump2AndAHalfGamePieceAndBalance);
      }
      else if (autoBoxSwitch1 == 3) {
        return new NonBumpThreeGamePieceAuto(s_Swerve, s_Arm, operator, nonBump3GamePiece);
      }      
      else if (autoBoxSwitch1 == 4) {
        return new NonBumpTwoGamePieceAndBalance(s_Swerve, s_Arm, operator, nonBump2GamePieceAndBalancePaths);
      }
      else if (autoBoxSwitch1 == 5) {
        return new ScoreOnly(s_Arm, s_Swerve);
      }
      else if (autoBoxSwitch1 == 6) {
        return new MoveBack(s_Swerve, s_Arm);
      }     
    }
    else if (autoBoxSwitch2 == 2) {
      if (autoBoxSwitch1 == 1) {
        return new MiddleScoreAndAutoBalance(s_Swerve, s_Arm);
      }
      else if (autoBoxSwitch1 == 2) {
        return new BumpTwoGamePieceAndBalance(s_Swerve, s_Arm, operator, bumpTwoGamePieceAndBalancePaths);
      }
      if (autoBoxSwitch1 == 3) {
        return new BumpThreeGamePiece(s_Swerve, s_Arm, operator, bumpThreeGamePiecePaths);
      }
      else if (autoBoxSwitch1 == 5) {
        return new ScoreOnly(s_Arm, s_Swerve);
      }
      else if (autoBoxSwitch1 == 6) {
        return new MoveBack(s_Swerve, s_Arm);
      }
    }
   return new DoNothing(s_Swerve, s_Arm);
    // An example command will be run in autonomous
   // return new exampleAuto(s_Swerve);
   
   //return new NonBumpTwoGamePieceAndBalance(s_Swerve, s_Arm, operator, nonBump2GamePieceAndBalancePaths);
   //return new NonBumpThreeGamePieceAuto(s_Swerve, s_Arm, operator, nonBump3GamePiece); //TODO place holder for now, replace once we have auto modes
  }
}
