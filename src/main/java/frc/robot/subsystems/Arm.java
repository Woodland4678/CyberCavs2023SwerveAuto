// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private CANSparkMax shoulderLeaderMotor;
  private CANSparkMax shoulderFollowerMotor;

  private CANSparkMax elbowLeaderMotor;
  private CANSparkMax elbowFollowerMotor;

  private CANSparkMax wristVerticalMotor;
  private CANSparkMax wristHorizontalMotor;

  private Solenoid clawPneumatic;

  private RelativeEncoder integratedShoulderEncoder;
  private RelativeEncoder integratedElbowEncoder;
  private RelativeEncoder integratedWristVerticalEncoder;
  private RelativeEncoder integratedWristHorizontalEncoder;

  private final SparkMaxPIDController shoulderController;
  private final SparkMaxPIDController elbowController;
  private final SparkMaxPIDController wristHorizontalController;
  private final SparkMaxPIDController wristVerticalController;

  private final DutyCycleEncoder shoulderAbsolute;
  private final DutyCycleEncoder elbowAbsolute;

  private int smartMovementState = 0;
  private double shoulderTargetAngle = 0;
  private double elbowTargetAngle = 0;

  private final TrapezoidProfile.Constraints shoulderConstraints =
      new TrapezoidProfile.Constraints(1.75, 0.75);
 // private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State shoulderSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State shoulderGoal = new TrapezoidProfile.State();
  private final SimpleMotorFeedforward shoulderFeedforward = new SimpleMotorFeedforward(1, 1.5);

  private final TrapezoidProfile.Constraints elbowConstraints =
      new TrapezoidProfile.Constraints(1.75, 0.75);
 // private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State elbowSetpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State elbowGoal = new TrapezoidProfile.State();
  private final SimpleMotorFeedforward elbowFeedforward = new SimpleMotorFeedforward(1, 1.5);
  
  /** Creates a new Arm. */
  public Arm() {
    shoulderLeaderMotor = new CANSparkMax(ArmConstants.shoulderLeaderMotorCanId, MotorType.kBrushless);
    shoulderFollowerMotor = new CANSparkMax(ArmConstants.shoulderFollowerMotorCanId, MotorType.kBrushless);

    elbowLeaderMotor = new CANSparkMax(ArmConstants.elbowLeaderMotorCanId, MotorType.kBrushless);
    elbowFollowerMotor = new CANSparkMax(ArmConstants.elbowFollowerMotorCanId, MotorType.kBrushless);

    wristVerticalMotor = new CANSparkMax(ArmConstants.wristVerticalCanId, MotorType.kBrushless);
    wristHorizontalMotor = new CANSparkMax(ArmConstants.wristHorizontalCanId, MotorType.kBrushless);
  
    shoulderFollowerMotor.follow(shoulderLeaderMotor);
    elbowFollowerMotor.follow(elbowLeaderMotor);
    
    clawPneumatic = new Solenoid(PneumaticsModuleType.REVPH, ArmConstants.pneumaticClawChannel);
    
    integratedShoulderEncoder = shoulderLeaderMotor.getEncoder();
    integratedElbowEncoder = elbowLeaderMotor.getEncoder();
    integratedWristHorizontalEncoder = wristHorizontalMotor.getEncoder();
    integratedWristVerticalEncoder = wristVerticalMotor.getEncoder();

    integratedShoulderEncoder.setPositionConversionFactor(Constants.ArmConstants.shoulderAngleConversionFactor);
    integratedElbowEncoder.setPositionConversionFactor(Constants.ArmConstants.elbowAngleConversionFactor);
    integratedWristHorizontalEncoder.setPositionConversionFactor(Constants.ArmConstants.wristHorizontalAngleConversionFactor);
    integratedWristVerticalEncoder.setPositionConversionFactor(Constants.ArmConstants.wristVerticalAngleConversionFactor);

    //shoulderLeaderMotor.setClosedLoopRampRate(0); //TODO may want this for acceleration control

    shoulderController = shoulderLeaderMotor.getPIDController();
    elbowController = elbowLeaderMotor.getPIDController();
    wristVerticalController = wristVerticalMotor.getPIDController();
    wristHorizontalController = wristHorizontalMotor.getPIDController();

    shoulderController.setP(Constants.ArmConstants.shoulderP);
    shoulderController.setI(Constants.ArmConstants.shoulderI);
    shoulderController.setD(Constants.ArmConstants.shoulderD);
    shoulderController.setFF(Constants.ArmConstants.shoulderFF);

    elbowController.setP(Constants.ArmConstants.elbowP);
    elbowController.setI(Constants.ArmConstants.elbowI);
    elbowController.setD(Constants.ArmConstants.elbowD);
    elbowController.setFF(Constants.ArmConstants.elbowFF);

    wristVerticalController.setP(Constants.ArmConstants.wristVerticalP);
    wristVerticalController.setI(Constants.ArmConstants.wristVerticalI);
    wristVerticalController.setD(Constants.ArmConstants.wristVerticalD);
    wristVerticalController.setFF(Constants.ArmConstants.wristVerticalFF);

    wristHorizontalController.setP(Constants.ArmConstants.wristHorizontalP);
    wristHorizontalController.setI(Constants.ArmConstants.wristHorizontalI);
    wristHorizontalController.setD(Constants.ArmConstants.wristHorizontalD);
    wristHorizontalController.setFF(Constants.ArmConstants.wristHorizontalFF);

    shoulderAbsolute = new DutyCycleEncoder(ArmConstants.shoulderEncoderAbsoluteID);
    elbowAbsolute = new DutyCycleEncoder(ArmConstants.elbowEncoderAbsoluteID);
    resetToAbsoluteEncoder();
  }
  public double getCurrentElbowPosition() {
    return integratedElbowEncoder.getPosition();
  }
  public double getCurrentShoulderPosition() {
    return integratedShoulderEncoder.getPosition();
  }
  public void ClawOpen(){
    clawPneumatic.set(true);
  }

  public void ClawClose(){
    clawPneumatic.set( false);
  }
  public void moveArmtrapezoidalInit(double currentElbowAngle, double currentShoulderAngle, double x, double y) {
    
    shoulderSetpoint.position = currentShoulderAngle;
    shoulderSetpoint.velocity = 0;
    elbowSetpoint.position = currentElbowAngle;
    elbowSetpoint.velocity = 0;
    double elbowAngle =  calcElbowAngle(x, y);
    double shoulderAngle = calcShoulderAngle(x, y, elbowAngle);
   // elbowAngle = shoulderAngle - elbowAngle; //get elbow angle relative to horizontal rather than relative to shoulder angle.
    elbowGoal.position = elbowAngle;
    elbowGoal.velocity = 0;

    shoulderGoal.position = shoulderAngle;
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

    shoulderController.setReference(shoulderSetpoint.position, com.revrobotics.CANSparkMax.ControlType.kPosition, 0, shoulderFeedforward.calculate(shoulderSetpoint.velocity)/12);
    elbowController.setReference(elbowSetpoint.position, com.revrobotics.CANSparkMax.ControlType.kPosition, 0, elbowFeedforward.calculate(elbowSetpoint.velocity)/12);
  }
  public void MoveArm(double x, double y) { //TODO we shouldn't recalculate angles each time, if we use this method change it
    double elbowAngle =  calcElbowAngle(x, y);
    double shoulderAngle = calcShoulderAngle(x, y, elbowAngle);
    //elbowAngle = shoulderAngle - elbowAngle; //get elbow angle relative to horizontal rather than relative to shoulder angle.
    shoulderController.setReference(shoulderAngle, com.revrobotics.CANSparkMax.ControlType.kPosition);
    elbowController.setReference(elbowAngle, com.revrobotics.CANSparkMax.ControlType.kPosition);
  }
  
  public void resetSmartMotionState() {
    smartMovementState = 0;
  }
  public void stopArm() {
    elbowLeaderMotor.stopMotor();
    shoulderLeaderMotor.stopMotor();
    wristHorizontalMotor.stopMotor();
    wristVerticalMotor.stopMotor();
  }
  private double calcElbowAngle(double x, double y) {
    return - Math.acos(((x * x) + (y * y) - (Math.pow(Constants.ArmConstants.shoulderLength, 2)) - (Math.pow(Constants.ArmConstants.elbowLength, 2))) / (2 * Constants.ArmConstants.shoulderLength * Constants.ArmConstants.elbowLength));
  }
  private double calcShoulderAngle(double x, double y, double calculatedElbowAngle) {
    return Math.atan(y / x) + Math.atan((Constants.ArmConstants.elbowLength * Math.sin(calculatedElbowAngle)) / (Constants.ArmConstants.elbowLength + Constants.ArmConstants.shoulderLength * Math.cos(calculatedElbowAngle)));
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
        shoulderController.setReference(shoulderTargetAngle, com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
        elbowController.setReference(elbowTargetAngle, com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
      break;
    }
  }

  public void MoveWristHorizontal(double position) {
    wristHorizontalController.setReference(position, com.revrobotics.CANSparkMax.ControlType.kPosition);
  }

  public void MoveWristVertical(double position) {
    wristVerticalController.setReference(position, com.revrobotics.CANSparkMax.ControlType.kPosition);
  }

  private void resetToAbsoluteEncoder() {
    integratedShoulderEncoder.setPosition(shoulderAbsolute.getAbsolutePosition() - ArmConstants.shoulderAngleOffset);
    integratedElbowEncoder.setPosition(elbowAbsolute.getAbsolutePosition() - ArmConstants.elbowAngleOffset);
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber( 
                  "Elbow Angle", integratedElbowEncoder.getPosition());
    SmartDashboard.putNumber( 
                  "Elbow Absolute", elbowAbsolute.getAbsolutePosition());
    SmartDashboard.putNumber( 
                  "Shoulder Absolute", shoulderAbsolute.getAbsolutePosition());
    SmartDashboard.putNumber( 
                  "Shoulder Angle", integratedShoulderEncoder.getPosition());
    SmartDashboard.putNumber( 
                  "Wrist Horizontal Angle", integratedWristHorizontalEncoder.getPosition());
    SmartDashboard.putNumber( 
                  "Wrist Vertical Angle", integratedWristVerticalEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
