// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.BaseAuto;
//import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.autos.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final Joystick driver = new Joystick(0);
  private static final Joystick operator = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton driverBtnY =
      new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton driverBtnLB =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton driverBtnRB =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton driverBtnA =
      new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton driverBtnB =
      new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton driverBtnX =
      new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton driverBtnRT = 
      new JoystickButton(driver, XboxController.Axis.kRightTrigger.value);
  private final JoystickButton driverBtnBack =
      new JoystickButton(driver, XboxController.Button.kBack.value);
  
  

  /* Operator Buttons */
  private final JoystickButton operatorBtnY =
      new JoystickButton(operator, 4);
  private final JoystickButton operatorBtnX =
      new JoystickButton(operator, 1);
  private final JoystickButton operatorBtnA =
      new JoystickButton(operator, 2);
  private final JoystickButton operatorBtnB =
      new JoystickButton(operator, 3);
  private final JoystickButton operatorBtnLB =
      new JoystickButton(operator, 5);
  private final JoystickButton operatorBtnRB =
      new JoystickButton(operator, 6);
  private final JoystickButton operatorBtnBack =
      new JoystickButton(operator, 9);
  private final JoystickButton operatorBtnStart =
      new JoystickButton(operator, 10);
  private final JoystickButton operatorBtnRT =
      new JoystickButton(operator, 8);

  /* Subsystems */
  private final SwerveDrive s_Swerve = new SwerveDrive();
  private final Intake intake = new Intake();
  private final Arm s_Arm = new Arm();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> driverBtnRT.getAsBoolean(),
            () -> driverBtnLB.getAsBoolean(),
            () -> driverBtnRB.getAsBoolean()));



    // Configure the button bindings
    configureButtonBindings();
  }
  private void configureButtonBindings() {
    /* Driver Buttons */
    driverBtnBack.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    driverBtnA.whileTrue(new UprightGrabSequence(s_Swerve, s_Arm));
    driverBtnX.onTrue(new InstantCommand(() -> s_Arm.openClaw()));
    driverBtnB.onTrue(new InstantCommand(() -> s_Arm.closeClaw()));
    driverBtnRB.whileTrue(new YeetCube(s_Arm));
    driverBtnY.whileTrue(new AutoScoreHigh(s_Arm, s_Swerve));
    //followTape.whileTrue(new FollowTape(s_Swerve, driver));
    //autoBalance.whileTrue(new AutoBalance(s_Swerve));
    //moveArmPos1.whileTrue(new OpenClaw(s_Arm, 1));
    //moveArmPos2.whileTrue(new OpenClaw(s_Arm, 2));
    //moveArmPos1.whileTrue(new ArmMovePIDOnly(s_Arm, 1));
    //moveArmPos2.whileTrue(new ArmMovePIDOnly(s_Arm, 2));

    /* Operator Buttons */
    operatorBtnY.onTrue(new MoveArm(s_Arm,Constants.ArmConstants.scoreConeHighPosition, operator));
    operatorBtnB.onTrue(new MoveArm(s_Arm,Constants.ArmConstants.scoreConeMediumPosition, operator));
    operatorBtnA.onTrue(new MoveArm(s_Arm,Constants.ArmConstants.scoreLowPosition, operator));
    operatorBtnX.onTrue(new MoveArm(s_Arm,Constants.ArmConstants.pickupPosition, operator));
    operatorBtnLB.onTrue(new MoveArm(s_Arm,Constants.ArmConstants.restPosition, operator));
    operatorBtnRB.onTrue(new GrabGamePiece(s_Arm));
    operatorBtnBack.onTrue(new CalibrateArm(s_Arm));
    operatorBtnRT.onTrue(new MoveArm(s_Arm,Constants.ArmConstants.grabUprightConePosition, operator));
  }

  public static Joystick getOperatorJoystick() {
    return operator;
  }
  public static Joystick getDriverJoystick() {
    return driver;
  }
  public void resetArmAngles() {
    s_Arm.resetToAbsoluteEncoder();
  }
  public void resetSwerveModuleAngles() {
    s_Swerve.resetSwerveModuleAngles();
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
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
   // return new exampleAuto(s_Swerve);
   return new BaseAuto(s_Swerve); //TODO place holder for now, replace once we have auto modes
  }
}
