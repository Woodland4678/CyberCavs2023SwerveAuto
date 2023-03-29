package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends CommandBase {
  private SwerveDrive s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(4.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(4.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(4.0);

  private SlewRateLimiter autoRotateLimiter = new SlewRateLimiter(2);

  PIDController rController = new PIDController(0.04, 0.0001, 0.005);
  PIDController rControllerLoading = new PIDController(rController.getP(), rController.getI(), rController.getD());

  Trigger turnToDrivers;
  Trigger turnToLoading;
  boolean robotCentric;

  double rotationVal = 0;

  public TeleopSwerve(
      SwerveDrive s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      boolean robotCentric,
      Trigger turnToDrivers,
      Trigger turnToLoading
      ) { //BooleanSupplier robotCentricSup
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentric = robotCentric;
    // this.robotCentricSup = robotCentricSup;
      
    this.turnToDrivers = turnToDrivers;
    this.turnToLoading = turnToLoading;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));

    if (turnToDrivers.getAsBoolean()) {    
      var degrees = s_Swerve.getYaw().getDegrees();        
      if (rController.getSetpoint() > 0 && degrees < 0) {
        degrees = 360 + degrees;
      }
      else if (rController.getSetpoint() < 0 && degrees > 0) {
        degrees = degrees - 360;
      }
      
      rotationVal = rController.calculate(degrees);      
    }
    else if (!turnToDrivers.getAsBoolean()) {
      if (s_Swerve.getYaw().getDegrees() < 0) {
        rController.setSetpoint(-180);
      }
      else {
        rController.setSetpoint(180);
      }
      rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    }
    if (turnToLoading.getAsBoolean()) {
      var degrees = s_Swerve.getYaw().getDegrees();        
      if (rControllerLoading.getSetpoint() > 0 && degrees < 0) {
        degrees = 360 + degrees;
      }
      else if (rControllerLoading.getSetpoint() < 0 && degrees > 0) {
        degrees = degrees - 360;
      }
      
      rotationVal = rControllerLoading.calculate(degrees);     
    }
    else if (!turnToLoading.getAsBoolean()) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        rControllerLoading.setSetpoint(-90);
      }
      else {
        rControllerLoading.setSetpoint(90);
      }
    }
    // else if (turnToLoading.getAsBoolean()) {
    //   rController.setSetpoint(0);
    //   var degrees = s_Swerve.getYaw().getDegrees();      
    //   if (degrees < 0) {
    //     degrees += 360;
    //   }
    //   rotationVal = rController.calculate(degrees);
    // }
    //else {
      
   // }

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentric, //!robotCentricSup.getAsBoolean()
        true);
  }
}