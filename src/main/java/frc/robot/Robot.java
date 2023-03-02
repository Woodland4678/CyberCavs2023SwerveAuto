// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.dense.row.decomposition.svd.SafeSvd_DDRM;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.config.CTREConfigs;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static CTREConfigs ctreConfigs;
  private RobotContainer m_robotContainer;
  double elbowKP;
  double elbowKI;
  double elbowKD;
  double elbowKIz;
  double elbowKFF;

  double shoulderKP;
  double shoulderKI;
  double shoulderKD;
  double shoulderKIz;
  double shoulderKFF;

  double wristPitchKP;
  double wristPitchKI;
  double wristPitchKD;
  double wristPitchKIz;
  double wristPitchKFF;

  double wristRollKP;
  double wristRollKI;
  double wristRollKD;
  double wristRollKIz;
  double wristRollKFF;

  int gamePieceMode = 0;
  // PWM port 0
    // Must be a PWM header, not MXP or DIO
    AddressableLED m_led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
    int m_rainbowFirstPixelHue = 0;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_led.setLength(m_ledBuffer.getLength());
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    CommandScheduler.getInstance().run();
    
    // Fill the buffer with a rainbow
    rainbow();
    // Set the LEDs
    m_led.setData(m_ledBuffer);
  }
  private void rainbow() {
    
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_robotContainer.resetArmAngles();
    m_robotContainer.resetSwerveModuleAngles();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    elbowKP = Constants.ArmConstants.elbowP; 
    elbowKI = Constants.ArmConstants.elbowI;
    elbowKD = Constants.ArmConstants.elbowD; 
    elbowKIz = 0; 
    elbowKFF = Constants.ArmConstants.elbowFF; 

    shoulderKP = Constants.ArmConstants.shoulderP; 
    shoulderKI = Constants.ArmConstants.shoulderI;
    shoulderKD = Constants.ArmConstants.shoulderD; 
    shoulderKIz = 0; 
    shoulderKFF = Constants.ArmConstants.shoulderFF; 

    wristPitchKP = Constants.ArmConstants.wristPitchP; 
    wristPitchKI = Constants.ArmConstants.wristPitchI;
    wristPitchKD = Constants.ArmConstants.wristPitchD; 
    wristPitchKIz = 0; 
    wristPitchKFF = Constants.ArmConstants.wristPitchFF; 

    wristRollKP = Constants.ArmConstants.wristRollP; 
    wristRollKI = Constants.ArmConstants.wristRollI;
    wristRollKD = Constants.ArmConstants.wristRollD; 
    wristRollKIz = 0; 
    wristRollKFF = Constants.ArmConstants.wristRollFF; 
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    /*Uncomment below lines to tune elbow and shoulder pids */
    /*SmartDashboard.putNumber("Elbow P Gain", elbowKP);
    SmartDashboard.putNumber("Elbow I Gain", elbowKI);
    SmartDashboard.putNumber("Elbow D Gain", elbowKD);
    SmartDashboard.putNumber("Elbow I Zone", elbowKIz);
    SmartDashboard.putNumber("Elbow Feed Forward", elbowKFF);

    SmartDashboard.putNumber("Shoulder P Gain", shoulderKP);
    SmartDashboard.putNumber("Shoulder I Gain", shoulderKI);
    SmartDashboard.putNumber("Shoulder D Gain", shoulderKD);
    SmartDashboard.putNumber("Shoulder I Zone", shoulderKIz);
    SmartDashboard.putNumber("Shoulder Feed Forward", shoulderKFF);*/

    SmartDashboard.putNumber("Wrist Pitch P Gain", wristPitchKP);
    SmartDashboard.putNumber("Wrist Pitch I Gain", wristPitchKI);
    SmartDashboard.putNumber("Wrist Pitch D Gain", wristPitchKD);
    SmartDashboard.putNumber("Wrist Pitch I Zone", wristPitchKIz);
    SmartDashboard.putNumber("Wrist Pitch Feed Forward", wristPitchKFF);

    SmartDashboard.putNumber("Wrist Roll P Gain", wristRollKP);
    SmartDashboard.putNumber("Wrist Roll I Gain", wristRollKI);
    SmartDashboard.putNumber("Wrist Roll D Gain", wristRollKD);
    SmartDashboard.putNumber("Wrist Roll I Zone", wristRollKIz);
    SmartDashboard.putNumber("Wrist Roll Feed Forward", wristRollKFF);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //m_robotContainer.moveShoulder(-0.10);
    //m_robotContainer.moveElbow(0.05);
    /*Uncomment the below lines to tune elbow and shoulder PIDS */
    /*double elbowP = SmartDashboard.getNumber("Elbow P Gain", 0);
    double elbowI = SmartDashboard.getNumber("Elbow I Gain", 0);
    double elbowD = SmartDashboard.getNumber("Elbow D Gain", 0);
    double elbowIZ = SmartDashboard.getNumber("Elbow I Zone", 0);
    double elbowFF = SmartDashboard.getNumber("Elbow Feed Forward", 0);

    double shoulderP = SmartDashboard.getNumber("Shoulder P Gain", 0);
    double shoulderI = SmartDashboard.getNumber("Shoulder I Gain", 0);
    double shoulderD = SmartDashboard.getNumber("Shoulder D Gain", 0);
    double shoulderIZ = SmartDashboard.getNumber("Shoulder I Zone", 0);
    double shoulderFF = SmartDashboard.getNumber("Shoulder Feed Forward", 0); */

    double wristPitchP = SmartDashboard.getNumber("Wrist Pitch P Gain", 0);
    double wristPitchI = SmartDashboard.getNumber("Wrist Pitch I Gain", 0);
    double wristPitchD = SmartDashboard.getNumber("Wrist Pitch D Gain", 0);
    double wristPitchIZ = SmartDashboard.getNumber("Wrist Pitch I Zone", 0);
    double wristPitchFF = SmartDashboard.getNumber("Wrist Pitch Feed Forward", 0);

    double wristRollP = SmartDashboard.getNumber("Wrist Roll P Gain", 0);
    double wristRollI = SmartDashboard.getNumber("Wrist Roll I Gain", 0);
    double wristRollD = SmartDashboard.getNumber("Wrist Roll D Gain", 0);
    double wristRollIZ = SmartDashboard.getNumber("Wrist Roll I Zone", 0);
    double wristRollFF = SmartDashboard.getNumber("Wrist Roll Feed Forward", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    /*if((elbowP != elbowKP)) { m_robotContainer.setElbowPIDF(elbowP, elbowI, elbowIZ, elbowD, elbowFF); elbowKP = elbowP; }
    if((elbowI != elbowKI)) { m_robotContainer.setElbowPIDF(elbowP, elbowI, elbowIZ, elbowD, elbowFF); elbowKI = elbowI; }
    if((elbowD != elbowKD)) { m_robotContainer.setElbowPIDF(elbowP, elbowI, elbowIZ, elbowD, elbowFF); elbowKD = elbowD; }
    if((elbowIZ != elbowIZ)) { m_robotContainer.setElbowPIDF(elbowP, elbowI, elbowIZ, elbowD, elbowFF); elbowKIz = elbowIZ; }
    if((elbowFF != elbowKFF)) { m_robotContainer.setElbowPIDF(elbowP, elbowI, elbowIZ, elbowD, elbowFF); elbowKFF = elbowFF; }
  
    if((shoulderP != shoulderKP)) { m_robotContainer.setShoulderPIDF(shoulderP, shoulderI, shoulderIZ, shoulderD, shoulderFF); shoulderKP = shoulderP; }
    if((shoulderI != shoulderKI)) { m_robotContainer.setShoulderPIDF(shoulderP, shoulderI, shoulderIZ, shoulderD, shoulderFF); shoulderKI = shoulderI; }
    if((shoulderD != shoulderKD)) { m_robotContainer.setShoulderPIDF(shoulderP, shoulderI, shoulderIZ, shoulderD, shoulderFF); shoulderKD = shoulderD; }
    if((shoulderIZ != shoulderKIz)) { m_robotContainer.setShoulderPIDF(shoulderP, shoulderI, shoulderIZ, shoulderD, shoulderFF); shoulderKIz = shoulderIZ; }
    if((shoulderFF != shoulderKFF)) { m_robotContainer.setShoulderPIDF(shoulderP, shoulderI, shoulderIZ, shoulderD, shoulderFF); shoulderKFF = shoulderFF; } 
 */
    if((wristPitchP != wristPitchKP)) { m_robotContainer.setWristPitchPIDF(wristPitchP, wristPitchI, wristPitchIZ, wristPitchD, wristPitchFF); wristPitchKP = wristPitchP; }
    if((wristPitchI != wristPitchKI)) { m_robotContainer.setWristPitchPIDF(wristPitchP, wristPitchI, wristPitchIZ, wristPitchD, wristPitchFF); wristPitchKI = wristPitchI; }
    if((wristPitchD != wristPitchKD)) { m_robotContainer.setWristPitchPIDF(wristPitchP, wristPitchI, wristPitchIZ, wristPitchD, wristPitchFF); wristPitchKD = wristPitchD; }
    if((wristPitchIZ != wristPitchKIz)) { m_robotContainer.setWristRollPIDF(wristPitchP, wristPitchI, wristPitchIZ, wristPitchD, wristPitchFF); wristPitchKIz = wristPitchIZ; }
    if((wristPitchFF != wristPitchKFF)) { m_robotContainer.setWristRollPIDF(wristPitchP, wristPitchI, wristPitchIZ, wristPitchD, wristPitchFF); wristPitchKFF = wristPitchFF; } 

    if((wristRollP != wristRollKP)) { m_robotContainer.setWristRollPIDF(wristRollP, wristRollI, wristRollIZ, wristRollD, wristRollFF); wristRollKP = wristRollP; }
    if((wristRollI != wristRollKI)) { m_robotContainer.setWristRollPIDF(wristRollP, wristRollI, wristRollIZ, wristRollD, wristRollFF); wristRollKI = wristRollI; }
    if((wristRollD != wristRollKD)) { m_robotContainer.setWristRollPIDF(wristRollP, wristRollI, wristRollIZ, wristRollD, wristRollFF); wristRollKD = wristRollD; }
    if((wristRollIZ != wristRollKIz)) { m_robotContainer.setWristRollPIDF(wristRollP, wristRollI, wristRollIZ, wristRollD, wristRollFF); wristRollKIz = wristRollIZ; }
    if((wristRollFF != wristRollKFF)) { m_robotContainer.setWristRollPIDF(wristRollP, wristRollI, wristRollIZ, wristRollD, wristRollFF); wristRollKFF = wristRollFF; } 
  }
  public int getGamePieceMode() {
    return gamePieceMode;
  }
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
