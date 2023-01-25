// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  /** Creates a new Intake. */
  public Intake() {
    leftMotor = new CANSparkMax(IntakeConstants.leftMotorCanId, MotorType.kBrushless);
    rightMotor = new CANSparkMax(IntakeConstants.rightMotorCanId, MotorType.kBrushless);
  }
  public void setLeftMotor(double speed) {
    leftMotor.set(-speed);
  }
  public void setRightMotor(double speed) {
    rightMotor.set(speed);
  }

  @Override
  public void periodic() {
    double xVal = RobotContainer.getOperatorJoystick().getX();
    double yVal = RobotContainer.getOperatorJoystick().getY();
    if (xVal < 0.05) {
      xVal = 0;
    }
    if (yVal < 0.05) {
      yVal = 0;
    }
    setLeftMotor(yVal - (0.75 * xVal));
    setRightMotor(yVal + (0.75 * xVal));

    // This method will be called once per scheduler run
  }
}
