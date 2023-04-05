// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.dense.row.decomposition.svd.SafeSvd_DDRM;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
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

  boolean isArmStartOkay = false;

  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    isArmStartOkay = false;
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
    // if (!isArmStartOkay) {
    //   m_robotContainer.resetArmAngles();
    //   isArmStartOkay = m_robotContainer.isArmStartOkay();
    //   m_robotContainer.setArmMayMove(false);
    // }
    // else {
    //   m_robotContainer.setArmMayMove(true);
    // }
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    
    CommandScheduler.getInstance().run();
    
    // Fill the buffer with a rainbow
    //rainbow();
    // Set the LEDs
    
  }  
  
  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {
    var bval = 0;
    m_robotContainer.resetArmAngles();
	// 0x01 is first set of LEDs (lower right).  Front right Swerve
	// 0x02 is second set (mid lower right). Shoulder encoder
	// 0x04 is third set (mid upper right). Elbow encoder
	// 0x08 is 4th set (upper right).  Rear right Swerve
	// 0x10 is 5th set (upper left). Rear left Swerve
	// 0x20 is 6th set (mid upper left).  Gyro
	// 0x40 is 7th set (mid lower left).  Limelight
	// 0x80 is 8th set (lower left).  Front left Swerve
    if (m_robotContainer.isElbowReady()){
		bval += 0x04; 
	}
    if (m_robotContainer.isShoulderReady()){
		bval += 0x02; 
	}
    if (m_robotContainer.isFrontLeftSwerveReady()){
		bval += 0x01; 
	}
    if (m_robotContainer.isFrontRightSwerveReady()){
		bval += 0x80; 
	}
    if (m_robotContainer.isBackLeftSwerveReady()){
		bval += 0x08; 
	}
    if (m_robotContainer.isBackRightSwerveReady()){
		bval += 0x10; 
	}
    if (m_robotContainer.isGyroReady()){
		bval += 0x20; 
	}
    if (m_robotContainer.isLimelightReady()){
		bval += 0x40; 
	}
  m_robotContainer.setLEDsForDiagnostics(bval);
  SmartDashboard.putBoolean("is Elbow Ready", m_robotContainer.isElbowReady());
  SmartDashboard.putBoolean("is shoulder Ready", m_robotContainer.isShoulderReady());
  SmartDashboard.putBoolean("is mod 0 Ready", m_robotContainer.isFrontLeftSwerveReady());
  SmartDashboard.putBoolean("is mod 1 Ready", m_robotContainer.isFrontRightSwerveReady());
  SmartDashboard.putBoolean("is mod 2 Ready", m_robotContainer.isBackLeftSwerveReady());
  SmartDashboard.putBoolean("is mod 3 Ready", m_robotContainer.isBackRightSwerveReady());
  SmartDashboard.putBoolean("is gyro Ready", m_robotContainer.isGyroReady());
  SmartDashboard.putBoolean("is limelight Ready", m_robotContainer.isLimelightReady());
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
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    SmartDashboard.putNumber("Elbow P Gain", elbowKP);
    SmartDashboard.putNumber("Elbow I Gain", elbowKI);
    SmartDashboard.putNumber("Elbow D Gain", elbowKD);
    SmartDashboard.putNumber("Elbow I Zone", elbowKIz);
    SmartDashboard.putNumber("Elbow Feed Forward", elbowKFF);

    // SmartDashboard.putNumber("Shoulder P Gain", shoulderKP);
    // SmartDashboard.putNumber("Shoulder I Gain", shoulderKI);
    // SmartDashboard.putNumber("Shoulder D Gain", shoulderKD);
    // SmartDashboard.putNumber("Shoulder I Zone", shoulderKIz);
    // SmartDashboard.putNumber("Shoulder Feed Forward", shoulderKFF);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //m_robotContainer.moveShoulder(-0.10);
    //m_robotContainer.moveElbow(0.05);
    double elbowP = SmartDashboard.getNumber("Elbow P Gain", 0);
    double elbowI = SmartDashboard.getNumber("Elbow I Gain", 0);
    double elbowD = SmartDashboard.getNumber("Elbow D Gain", 0);
    double elbowIZ = SmartDashboard.getNumber("Elbow I Zone", 0);
    double elbowFF = SmartDashboard.getNumber("Elbow Feed Forward", 0);

    // double shoulderP = SmartDashboard.getNumber("Shoulder P Gain", 0);
    // double shoulderI = SmartDashboard.getNumber("Shoulder I Gain", 0);
    // double shoulderD = SmartDashboard.getNumber("Shoulder D Gain", 0);
    // double shoulderIZ = SmartDashboard.getNumber("Shoulder I Zone", 0);
    // double shoulderFF = SmartDashboard.getNumber("Shoulder Feed Forward", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((elbowP != elbowKP)) { m_robotContainer.setElbowPIDF(elbowP, elbowI, elbowIZ, elbowD, elbowFF); elbowKP = elbowP; }
    // if((elbowI != elbowKI)) { m_robotContainer.setElbowPIDF(elbowP, elbowI, elbowIZ, elbowD, elbowFF); elbowKI = elbowI; }
    // if((elbowD != elbowKD)) { m_robotContainer.setElbowPIDF(elbowP, elbowI, elbowIZ, elbowD, elbowFF); elbowKD = elbowD; }
    // if((elbowIZ != elbowIZ)) { m_robotContainer.setElbowPIDF(elbowP, elbowI, elbowIZ, elbowD, elbowFF); elbowKIz = elbowIZ; }
    // if((elbowFF != elbowKFF)) { m_robotContainer.setElbowPIDF(elbowP, elbowI, elbowIZ, elbowD, elbowFF); elbowKFF = elbowFF; }
  
    if((elbowP != elbowKP)) { m_robotContainer.setSwerveRotationPID(elbowP, elbowI, elbowD); elbowKP = elbowP; }
    if((elbowI != elbowKI)) { m_robotContainer.setSwerveRotationPID(elbowP, elbowI, elbowD); elbowKI = elbowI; }
    if((elbowD != elbowKD)) { m_robotContainer.setSwerveRotationPID(elbowP, elbowI, elbowD); elbowKD = elbowD; }
   // if((elbowIZ != elbowIZ)) { m_robotContainer.setElbowPIDF(elbowP, elbowI, elbowIZ, elbowD, elbowFF); elbowKIz = elbowIZ; }
   // if((elbowFF != elbowKFF)) { m_robotContainer.setElbowPIDF(elbowP, elbowI, elbowIZ, elbowD, elbowFF); elbowKFF = elbowFF; }

    // if((shoulderP != shoulderKP)) { m_robotContainer.setShoulderPIDF(shoulderP, shoulderI, shoulderIZ, shoulderD, shoulderFF); shoulderKP = shoulderP; }
    // if((shoulderI != shoulderKI)) { m_robotContainer.setShoulderPIDF(shoulderP, shoulderI, shoulderIZ, shoulderD, shoulderFF); shoulderKI = shoulderI; }
    // if((shoulderD != shoulderKD)) { m_robotContainer.setShoulderPIDF(shoulderP, shoulderI, shoulderIZ, shoulderD, shoulderFF); shoulderKD = shoulderD; }
    // if((shoulderIZ != shoulderKIz)) { m_robotContainer.setShoulderPIDF(shoulderP, shoulderI, shoulderIZ, shoulderD, shoulderFF); shoulderKIz = shoulderIZ; }
    // if((shoulderFF != shoulderKFF)) { m_robotContainer.setShoulderPIDF(shoulderP, shoulderI, shoulderIZ, shoulderD, shoulderFF); shoulderKFF = shoulderFF; } 
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
