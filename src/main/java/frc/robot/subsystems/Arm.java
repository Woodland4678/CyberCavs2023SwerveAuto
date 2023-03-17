// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.LEDModes;

public class Arm extends SubsystemBase {
  private CANSparkMax shoulderLeaderMotor;
  private CANSparkMax shoulderFollowerMotor;

  private CANSparkMax elbowLeaderMotor;
  private CANSparkMax elbowFollowerMotor;

  private CANSparkMax wristPitchMotor;
  private CANSparkMax wristRollMotor;

  private DoubleSolenoid clawSolenoid1;
  private DoubleSolenoid clawSolenoid2;

  private RelativeEncoder integratedShoulderEncoder;
  private RelativeEncoder integratedElbowEncoder;
  private RelativeEncoder integratedwristPitchEncoder;
  private RelativeEncoder integratedwristRollEncoder;

  private final SparkMaxPIDController shoulderController;
  private final SparkMaxPIDController elbowController;
  private final SparkMaxPIDController wristRollController;
  private final SparkMaxPIDController wristPitchController;

  private final DutyCycleEncoder shoulderAbsolute;
  private final DutyCycleEncoder elbowAbsolute;

  private int smartMovementState = 0;
  private double shoulderTargetAngle = 0;
  private double elbowTargetAngle = 0;

  private DigitalInput elbowTopLimitSwitch;
  private DigitalInput elbowBottomLimitSwitch;
  private DigitalInput shoulderTopLimitSwitch;
  private DigitalInput shoulderBottomLimitSwitch;

  private DigitalInput wristPitchLimitSwitch;

  AnalogInput wristRollCal = new AnalogInput(0);

  private boolean armMayMove = false;

  boolean isClawClosed = true;
  int clawClosedCnt = 0;

  private int calibrateWristState = 0;

  AddressableLED m_led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
  int m_rainbowFirstPixelHue = 0;

  private final TrapezoidProfile.Constraints shoulderConstraints =
      new TrapezoidProfile.Constraints(1.75, 0.75);
 // private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State shoulderSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State shoulderGoal = new TrapezoidProfile.State();
  //private final SimpleMotorFeedforward shoulderFeedforward = new SimpleMotorFeedforward(1, 1.5);
  private final ArmFeedforward shoulderFeedforward = new ArmFeedforward(smartMovementState, shoulderTargetAngle, elbowTargetAngle);

  private final TrapezoidProfile.Constraints elbowConstraints =
      new TrapezoidProfile.Constraints(1.75, 0.75);
 // private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State elbowSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State elbowGoal = new TrapezoidProfile.State();
  //private final SimpleMotorFeedforward elbowFeedforward = new SimpleMotorFeedforward(1, 1.5);
  private final ArmFeedforward elbowFeedForward = new ArmFeedforward(0.12139, 0.17521, 0.023944);
  

  public int gamePieceMode = Constants.ArmConstants.coneMode;
  int blinkCnt = 0;
  int blinkInterval = 10;

  LEDModes LEDMode;
  
  /** Creates a new Arm. */
  public Arm() {
    shoulderLeaderMotor = new CANSparkMax(ArmConstants.shoulderLeaderMotorCanId, MotorType.kBrushless);
    shoulderFollowerMotor = new CANSparkMax(ArmConstants.shoulderFollowerMotorCanId, MotorType.kBrushless);

    elbowLeaderMotor = new CANSparkMax(ArmConstants.elbowLeaderMotorCanId, MotorType.kBrushless);
    elbowFollowerMotor = new CANSparkMax(ArmConstants.elbowFollowerMotorCanId, MotorType.kBrushless);

    wristPitchMotor = new CANSparkMax(ArmConstants.wristPitchCanId, MotorType.kBrushless);
    wristRollMotor = new CANSparkMax(ArmConstants.wristRollCanId, MotorType.kBrushless);
  
    shoulderFollowerMotor.follow(shoulderLeaderMotor, true);
    elbowFollowerMotor.follow(elbowLeaderMotor);
    
    //clawSolenoid = new Solenoid(PneumaticsModuleType.REVPH, ArmConstants.pneumaticClawChannel);
    clawSolenoid1 = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 8, 9);
    clawSolenoid2 = new DoubleSolenoid(1, PneumaticsModuleType.REVPH, 10, 11);

    integratedShoulderEncoder = shoulderLeaderMotor.getEncoder();
    integratedElbowEncoder = elbowLeaderMotor.getEncoder();
    integratedwristRollEncoder = wristRollMotor.getEncoder();
    integratedwristPitchEncoder = wristPitchMotor.getEncoder();

    integratedShoulderEncoder.setPositionConversionFactor(Constants.ArmConstants.shoulderAngleConversionFactor);
    integratedElbowEncoder.setPositionConversionFactor(Constants.ArmConstants.elbowAngleConversionFactor);
    integratedwristRollEncoder.setPositionConversionFactor(Constants.ArmConstants.wristRollAngleConversionFactor);
    integratedwristPitchEncoder.setPositionConversionFactor(Constants.ArmConstants.wristPitchAngleConversionFactor);

    //elbowTopLimitSwitch = new DigitalInput(Constants.ArmConstants.elbowTopLimitSwitchChannel);
    //elbowBottomLimitSwitch = new DigitalInput(Constants.ArmConstants.elbowBottomLimitSwitchChannel);
   // shoulderTopLimitSwitch = new DigitalInput(Constants.ArmConstants.shoulderTopLimitSwitchChannel);
    //shoulderBottomLimitSwitch = new DigitalInput(Constants.ArmConstants.shoulderBottomLimitSwitchChannel);

    wristPitchLimitSwitch = new DigitalInput(Constants.ArmConstants.wristPitchLimitSwitchChannel);
    elbowLeaderMotor.setClosedLoopRampRate(0.5); //TODO may want this for acceleration control
    wristRollMotor.setClosedLoopRampRate(0.05);

    shoulderController = shoulderLeaderMotor.getPIDController();
    elbowController = elbowLeaderMotor.getPIDController();
    wristPitchController = wristPitchMotor.getPIDController();
    wristRollController = wristRollMotor.getPIDController();

    shoulderController.setP(Constants.ArmConstants.shoulderP);
    shoulderController.setI(Constants.ArmConstants.shoulderI);
    shoulderController.setD(Constants.ArmConstants.shoulderD);
    shoulderController.setFF(Constants.ArmConstants.shoulderFF);

    elbowController.setP(Constants.ArmConstants.elbowP);
    elbowController.setI(Constants.ArmConstants.elbowI);
    elbowController.setD(Constants.ArmConstants.elbowD);
    elbowController.setFF(Constants.ArmConstants.elbowFF);

    wristPitchController.setP(Constants.ArmConstants.wristPitchP);
    wristPitchController.setI(Constants.ArmConstants.wristPitchI);
    wristPitchController.setD(Constants.ArmConstants.wristPitchD);
    wristPitchController.setFF(Constants.ArmConstants.wristPitchFF);

    wristRollController.setP(Constants.ArmConstants.wristRollP);
    wristRollController.setI(Constants.ArmConstants.wristRollI);
    wristRollController.setD(Constants.ArmConstants.wristRollD);
    wristRollController.setFF(Constants.ArmConstants.wristRollFF);

    shoulderAbsolute = new DutyCycleEncoder(ArmConstants.shoulderEncoderAbsoluteID);
    elbowAbsolute = new DutyCycleEncoder(ArmConstants.elbowEncoderAbsoluteID);


    shoulderController.setOutputRange(-1, 1);
    elbowController.setOutputRange(-1, 1);
    //wristRollController.setOutputRange(elbowTargetAngle, smartMovementState)

    shoulderLeaderMotor.setSmartCurrentLimit(40);
    elbowLeaderMotor.setSmartCurrentLimit(40);
    elbowFollowerMotor.setSmartCurrentLimit(40);
    shoulderFollowerMotor.setSmartCurrentLimit(40);

    wristRollMotor.setSmartCurrentLimit(8);
    //shoulderAbsolute.setDistancePerRotation(360);
    //elbowAbsolute.setDistancePerRotation(360);
    integratedwristPitchEncoder.setPosition(0);
    integratedwristRollEncoder.setPosition(0);
    elbowLeaderMotor.burnFlash();
    shoulderLeaderMotor.burnFlash();
    resetToAbsoluteEncoder();

    m_led.setLength(m_ledBuffer.getLength());
    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    LEDMode = LEDModes.OFF;
  
  }
  public double getCurrentElbowPosition() {
    return integratedElbowEncoder.getPosition();
  }
  public double getCurrentShoulderPosition() {
    return integratedShoulderEncoder.getPosition();
  }
  public void openClaw(){
    clawSolenoid1.set(Value.kReverse);
    clawSolenoid2.set(Value.kReverse);
    isClawClosed = false;
  }

  public void closeClaw(){
    clawSolenoid1.set(Value.kForward);
    clawSolenoid2.set(Value.kForward);
    isClawClosed = true;
  }
  public int getGamePieceMode() {
    return gamePieceMode;
  }
  public void setArmCanMove(boolean canMove) {
    armMayMove = canMove;
  }
  public void setGamePieceMode(int mode) {
    gamePieceMode = mode;
  }
  public void moveArmtrapezoidalInit(double currentElbowAngle, double currentShoulderAngle, double x, double y) {
    
    shoulderSetpoint.position = currentShoulderAngle;
    shoulderSetpoint.velocity = 0;
    elbowSetpoint.position = currentElbowAngle;
    elbowSetpoint.velocity = 0;
    double elbowAngle =  calcElbowAngle(x, y);
    double shoulderAngle = calcShoulderAngle(x, y, elbowAngle);
   // elbowAngle = shoulderAngle - elbowAngle; //get elbow angle relative to horizontal rather than relative to shoulder angle.
    elbowGoal.position = -elbowAngle / 2;
    elbowGoal.velocity = 0;

    shoulderGoal.position = (90 - shoulderAngle) / 2;
    shoulderGoal.velocity = 0;
  }
  public void moveArmtrapezoidal() {
    //double elbowAngle =  - Math.acos(((x * x) + (y * y) - (Math.pow(Constants.ArmConstants.shoulderLength, 2)) - (Math.pow(Constants.ArmConstants.elbowLength, 2))) / (2 * Constants.ArmConstants.shoulderLength * Constants.ArmConstants.elbowLength));
   // double shoulderAngle = Math.atan(y / x) + Math.atan((Constants.ArmConstants.elbowLength * Math.sin(elbowAngle)) / (Constants.ArmConstants.elbowLength + Constants.ArmConstants.shoulderLength * Math.cos(elbowAngle)));
   // elbowAngle = shoulderAngle - elbowAngle; //get elbow angle relative to horizontal rather than relative to shoulder angle.
    
    var shoulderProfile = new TrapezoidProfile(shoulderConstraints, shoulderGoal, shoulderSetpoint);
    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    shoulderSetpoint = shoulderProfile.calculate(0.02);

    var elbowProfile = new TrapezoidProfile(elbowConstraints, elbowGoal, elbowSetpoint);
    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    elbowSetpoint = elbowProfile.calculate(0.02);

    shoulderController.setReference(shoulderSetpoint.position, com.revrobotics.CANSparkMax.ControlType.kPosition, 0, shoulderFeedforward.calculate(integratedShoulderEncoder.getPosition(),shoulderSetpoint.velocity)/12);
    elbowController.setReference(elbowSetpoint.position, com.revrobotics.CANSparkMax.ControlType.kPosition, 0, elbowFeedForward.calculate(90 - integratedShoulderEncoder.getPosition(),elbowSetpoint.velocity)/12);
  }
  public boolean isArmStartOkay() {
    if ((90 - integratedShoulderEncoder.getPosition()) > 105 && (90 - integratedShoulderEncoder.getPosition() < 132)) {
      if (-integratedElbowEncoder.getPosition() > -175 && -integratedElbowEncoder.getPosition() < -165) {
        return true;
      }
    }
    return false;
  }
  public double MoveArm(ArmPosition targetPos) { //TODO we shouldn't recalculate angles each time, if we use this method change it
    
    double elbowAngle = calcElbowAngle(targetPos.xTarget, targetPos.yTarget);
    double shoulderAngle = calcShoulderAngle(targetPos.xTarget, targetPos.yTarget, elbowAngle);
    double currentX = getCurrentXPosition();
    double currentY = getCurrentYPosition();
    
    SmartDashboard.putNumber( 
                  "Shoulder target angle lol", shoulderAngle);
    SmartDashboard.putNumber( 
                  "Elbow target angle lol", elbowAngle);
    
    //Check if we might be in the danger zone
    // if (currentY < Constants.ArmConstants.armExclusionY) {
    //   if (currentX < Constants.ArmConstants.armExclusionXMax && currentX > Constants.ArmConstants.armExclusionXMin) {
    //     //In danger zone
    //     if (currentX < targetPos.xTarget) { //moving outward stop the shoulder to allow the elbow to get up high enough
    //       shoulderLeaderMotor.stopMotor();
    //     }
    //     else { //stop the elbow to let the shoulder get back far enough
    //       elbowLeaderMotor.stopMotor();
    //     }
    //   }      
    // }
    //if we're not in the danger zone set the motors
    //else {
      if (shoulderAngle >= 45 && shoulderAngle <= 135 && elbowAngle <= 0 && elbowAngle >= -175 ) {
        moveToAngle(shoulderAngle, elbowAngle);
      }
      if (currentX > 17 || targetPos.wristPitchTarget > -30) {
        wristPitchController.setReference(targetPos.wristPitchTarget, com.revrobotics.CANSparkMax.ControlType.kPosition);
        wristRollController.setReference(targetPos.wristRollTarget, com.revrobotics.CANSparkMax.ControlType.kPosition);
      }
      else {
        wristRollMotor.stopMotor();
        wristPitchMotor.stopMotor();
      }
      
    
      
    //}
    return Math.sqrt((Math.pow(currentX - targetPos.xTarget,2)) + (Math.pow(currentY - targetPos.yTarget, 2))); //returns distance to target
  }
  public boolean isClawClosed() {
    return isClawClosed;
  }
  public void moveToAngle(double shoulderAngle, double elbowAngle) {
    shoulderController.setReference((90 - shoulderAngle) , com.revrobotics.CANSparkMax.ControlType.kPosition); //90 - inverse calc
    elbowController.setReference(-elbowAngle, com.revrobotics.CANSparkMax.ControlType.kPosition);
  }
  public void setElbowPIDF(double p, double i, double iZone, double d, double f) {
    elbowController.setP(p);
    elbowController.setI(i);
    elbowController.setIZone(iZone);
    elbowController.setD(d);
    elbowController.setFF(f);
  }
  public void setShoulderPIDF(double p, double i, double iZone, double d, double f) {
    shoulderController.setP(p);
    shoulderController.setI(i);
    shoulderController.setIZone(iZone);
    shoulderController.setD(d);
    shoulderController.setFF(f);
  }
  public void setWristPitchPIDF(double p, double i, double iZone, double d, double f) {
    wristPitchController.setP(p);
    wristPitchController.setI(i);
    wristPitchController.setIZone(iZone);
    wristPitchController.setD(d);
    wristPitchController.setFF(f);
  }
  public void setWristRollPIDF(double p, double i, double iZone, double d, double f) {
    wristRollController.setP(p);
    wristRollController.setI(i);
    wristRollController.setIZone(iZone);
    wristRollController.setD(d);
    wristRollController.setFF(f);
  }
  public void resetSmartMotionState() {
    smartMovementState = 0;
  }
  public void stopArm() {
    elbowLeaderMotor.stopMotor();
    shoulderLeaderMotor.stopMotor();
    wristRollMotor.stopMotor();
    wristPitchMotor.stopMotor();
  }
  private double calcElbowAngle(double x, double y) {
    // double term1 = (x*x + y*y - (Constants.ArmConstants.shoulderLength * Constants.ArmConstants.shoulderLength) - (Constants.ArmConstants.elbowLength * Constants.ArmConstants.elbowLength)/(2 * Constants.ArmConstants.shoulderLength * Constants.ArmConstants.elbowLength));
	  // if (term1 < -1.0) {
    //   term1 = -1.0;
    // }
    // if (term1 > 1.0) {
		//   term1 = 1.0;
    // }
    // return Math.toDegrees(-Math.acos(term1));


    return Math.toDegrees(- Math.acos(((x * x) + (y * y) - (Math.pow(Constants.ArmConstants.shoulderLength, 2)) - (Math.pow(Constants.ArmConstants.elbowLength, 2))) / (2 * Constants.ArmConstants.shoulderLength * Constants.ArmConstants.elbowLength)));
  }
  private double calcShoulderAngle(double x, double y, double calculatedElbowAngle) {
    return Math.toDegrees(Math.atan(y / x) - Math.atan((Constants.ArmConstants.elbowLength * Math.sin(Math.toRadians(calculatedElbowAngle))) / (Constants.ArmConstants.elbowLength + Constants.ArmConstants.shoulderLength * Math.cos(Math.toRadians(calculatedElbowAngle)))));
  }
  public void MoveArmSmartMotion(double x, double y, double accelShoulder, double maxVelocityShoulder, double accelElbow, double maxVelocityElbow) {
    switch (smartMovementState) {
      case 0:
        shoulderController.setSmartMotionMaxVelocity(maxVelocityShoulder, 0);
        //shoulderController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        shoulderController.setSmartMotionMaxAccel(accelShoulder, 0);
        //shoulderController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        elbowController.setSmartMotionMaxVelocity(maxVelocityElbow, 0);
        elbowController.setSmartMotionMaxAccel(accelElbow, 0);

        elbowTargetAngle = calcElbowAngle(x, y);
        shoulderTargetAngle = calcShoulderAngle(x, y, elbowTargetAngle);
        //elbowTargetAngle =   - Math.acos(((x * x) + (y * y) - (Math.pow(Constants.ArmConstants.shoulderLength, 2)) - (Math.pow(Constants.ArmConstants.elbowLength, 2))) / (2 * Constants.ArmConstants.shoulderLength * Constants.ArmConstants.elbowLength));
        //shoulderTargetAngle = Math.atan(y / x) + Math.atan((Constants.ArmConstants.elbowLength * Math.sin(elbowTargetAngle)) / (Constants.ArmConstants.elbowLength + Constants.ArmConstants.shoulderLength * Math.cos(elbowTargetAngle)));
        //elbowTargetAngle = shoulderTargetAngle - elbowTargetAngle; //get the elbow target relative to horizontal rather than relative to the shoulder angle
        smartMovementState++;
      break;
      case 1:
        shoulderController.setReference((90 - shoulderTargetAngle) / 2, com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
        elbowController.setReference(-elbowTargetAngle / 2, com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
      break;
    }
  }

  public void MovewristRoll(double position) {
    wristRollController.setReference(position, com.revrobotics.CANSparkMax.ControlType.kPosition);
  }

  public void MovewristPitch(double position) {
    wristPitchController.setReference(position, com.revrobotics.CANSparkMax.ControlType.kPosition);
  }
  public void resetCalibrateWristState() {
    calibrateWristState = 0;
  }
  public boolean calibrateWrist() {
    switch(calibrateWristState) {
      case 0:
        wristPitchMotor.set(0.1);
        if (!wristPitchLimitSwitch.get()) {
          calibrateWristState++;
        }
      break;
      case 1:
        wristPitchMotor.set(-0.1);
        if (wristPitchLimitSwitch.get()) {
          calibrateWristState++;
        }
      break;
      case 2:
        wristPitchMotor.stopMotor();
        integratedwristPitchEncoder.setPosition(0);
        calibrateWristState = 0;
        return true;
    }
    return false;
  }
  public void wristRollManual(double speed) {
    if (integratedwristRollEncoder.getPosition() > 720 && speed > 0) {
      wristRollMotor.stopMotor();
    }
    else if (integratedwristRollEncoder.getPosition() < -720 && speed < 0) {
      wristRollMotor.stopMotor();
    } 
    else {
      wristRollMotor.set(speed);
    }
  }
  public void wristPitchManual(double speed) {
    if (integratedwristRollEncoder.getPosition() <= 0 && speed > 0) {
      wristPitchMotor.stopMotor();
    }
    else if (integratedwristRollEncoder.getPosition() < -150 && speed < 0) {
      wristPitchMotor.stopMotor();
    } 
    else {
      wristPitchMotor.set(speed);
    }
  }
  public void resetToAbsoluteEncoder() {
    integratedShoulderEncoder.setPosition(((shoulderAbsolute.getAbsolutePosition()) * 360) - ArmConstants.shoulderAngleOffset);
    integratedElbowEncoder.setPosition(((elbowAbsolute.getAbsolutePosition()) * 360) - ArmConstants.elbowAngleOffset);
  }
  public double getCurrentXPosition() {
    double shoulderLength = Constants.ArmConstants.shoulderLength;
    double elbowLength = Constants.ArmConstants.elbowLength;
    double shoulderAngle = 90 - integratedShoulderEncoder.getPosition();
    double elbowAngle = integratedElbowEncoder.getPosition();
    return shoulderLength * Math.cos(Math.toRadians(shoulderAngle)) + elbowLength * Math.cos(Math.toRadians((shoulderAngle) - elbowAngle));
  }
  public double getCurrentYPosition() {
    double shoulderLength = Constants.ArmConstants.shoulderLength;
    double elbowLength = Constants.ArmConstants.elbowLength;
    double shoulderAngle = 90 - integratedShoulderEncoder.getPosition();
    double elbowAngle = integratedElbowEncoder.getPosition();
    return (shoulderLength * Math.sin(Math.toRadians(shoulderAngle)) + elbowLength * Math.sin(Math.toRadians((shoulderAngle) - elbowAngle)));
  }
  public double getCurrentWristRollPosition() {
    return integratedwristRollEncoder.getPosition();
  }
  public void runShoulderMotor(double runSpeed) {
    shoulderLeaderMotor.set(runSpeed);
  }
  public void runElbowMotor(double runSpeed) {
    elbowLeaderMotor.set(runSpeed);
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
  public void setLEDs(int r, int g, int b) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Set the value
      m_ledBuffer.setRGB(i, r, g, b);
    }
  }
  public void coneMode() {
    //setLEDs(255, 255, 0);
    setGamePieceMode(1);
    LEDMode = LEDModes.SOLIDYELLOW;
    gamePieceMode = 1;
  }
  public void cubeMode() {
    //setLEDs(75, 0, 130);
    LEDMode = LEDModes.SOLIDPURPLE;
    setGamePieceMode(Constants.ArmConstants.cubeMode); //TODO change this back to cubes
  }
  public void setLEDMode(LEDModes mode) {
    LEDMode = mode;
  }
  @Override
  public void periodic() {

    switch (LEDMode) {
      case OFF:
        setLEDs(0, 0, 0);
      break;
      case BLINKGREEN:
        if (blinkCnt > blinkInterval) {
          setLEDs(0, 255, 0);
          blinkCnt = 0;
        }
        else {
          blinkCnt++;
          setLEDs(0, 0, 0);
        }

      break;
      case SOLIDPURPLE:
        setLEDs(75, 0, 130);
      break;
      case SOLIDYELLOW:
        setLEDs(255, 255, 0);
      break;
    }
    m_led.setData(m_ledBuffer);
    //resetToAbsoluteEncoder();
    SmartDashboard.putNumber( 
                  "Elbow Angle", -integratedElbowEncoder.getPosition());
    SmartDashboard.putNumber( 
                  "Elbow Absolute", elbowAbsolute.getAbsolutePosition() * 360);
    SmartDashboard.putNumber( 
                  "Shoulder Absolute", (shoulderAbsolute.getAbsolutePosition()) * 360);
    SmartDashboard.putNumber( 
                  "Shoulder Angle", 90 - integratedShoulderEncoder.getPosition());
    SmartDashboard.putNumber(
                  "Wrist Horizontal Angle", integratedwristRollEncoder.getPosition());
    SmartDashboard.putNumber( 
                  "Wrist Vertical Angle", integratedwristPitchEncoder.getPosition());
    SmartDashboard.putNumber( 
                  "Arm X Position", getCurrentXPosition());
    SmartDashboard.putNumber( 
                  "Arm Y Position", getCurrentYPosition());
    SmartDashboard.putBoolean( 
                  "Wrist Pitch Limit Switch", wristPitchLimitSwitch.get());
    SmartDashboard.putNumber( 
                  "Game Piece Mode", getGamePieceMode());
    SmartDashboard.putNumber( 
                  "Wrist Roll Cal", wristRollCal.getValue());
    // This method will be called once per scheduler run
    // if (elbowTopLimitSwitch.get() || elbowBottomLimitSwitch.get()) {
    //   elbowLeaderMotor.stopMotor();
    // }
    // if (shoulderTopLimitSwitch.get() || shoulderBottomLimitSwitch.get()) {
    //   shoulderLeaderMotor.stopMotor();
    // }
  }
}
