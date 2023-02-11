// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
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

  public void ClawOpen(){
    clawPneumatic.set(true);
  }

  public void ClawClose(){
    clawPneumatic.set( false);
  }

  public void MoveArm(int x, int y) {
    double elbowAngle =  - Math.acos(((x * x) + (y * y) - (Math.pow(Constants.ArmConstants.shoulderLength, 2)) - (Math.pow(Constants.ArmConstants.elbowLength, 2))) / (2 * Constants.ArmConstants.shoulderLength * Constants.ArmConstants.elbowLength));
    double shoulderAngle = Math.atan(y / x) + Math.atan((Constants.ArmConstants.elbowLength * Math.sin(elbowAngle)) / (Constants.ArmConstants.elbowLength + Constants.ArmConstants.shoulderLength * Math.cos(elbowAngle)));
    elbowAngle = shoulderAngle - elbowAngle; //get elbow angle relative to horizontal rather than relative to shoulder angle.
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
  public void MoveArmSmartMotion(int x, int y, double accelShoulder, double maxVelocityShoulder, double accelElbow, double maxVelocityElbow) {
    switch (smartMovementState) {
      case 0:
        shoulderController.setSmartMotionMaxVelocity(maxVelocityShoulder, 0);
        //shoulderController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        shoulderController.setSmartMotionMaxAccel(accelShoulder, 0);
        //shoulderController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        elbowController.setSmartMotionMaxVelocity(maxVelocityElbow, 0);
        elbowController.setSmartMotionMaxAccel(accelElbow, 0);

        elbowTargetAngle =   - Math.acos(((x * x) + (y * y) - (Math.pow(Constants.ArmConstants.shoulderLength, 2)) - (Math.pow(Constants.ArmConstants.elbowLength, 2))) / (2 * Constants.ArmConstants.shoulderLength * Constants.ArmConstants.elbowLength));
        shoulderTargetAngle = Math.atan(y / x) + Math.atan((Constants.ArmConstants.elbowLength * Math.sin(elbowTargetAngle)) / (Constants.ArmConstants.elbowLength + Constants.ArmConstants.shoulderLength * Math.cos(elbowTargetAngle)));
        elbowTargetAngle = shoulderTargetAngle - elbowTargetAngle; //get the elbow target relative to horizontal rather than relative to the shoulder angle
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
    // This method will be called once per scheduler run
  }
}
