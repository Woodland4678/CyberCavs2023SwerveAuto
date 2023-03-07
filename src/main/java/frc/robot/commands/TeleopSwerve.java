package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  PIDController rController = new PIDController(Constants.Swerve.autoDriveRP, Constants.Swerve.autoDriveRI, Constants.Swerve.autoDriveRD);

  BooleanSupplier turnToDrivers;
  BooleanSupplier turnToLoading;

  double rotationVal = 0;

  public TeleopSwerve(
      SwerveDrive s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier turnToDrivers,
      BooleanSupplier turnToLoading) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;

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
      rController.setSetpoint(180);
      var degrees = s_Swerve.getYaw().getDegrees();      
      rotationVal = rController.calculate(degrees);
    }
    else if (turnToLoading.getAsBoolean()) {
      rController.setSetpoint(0);
      var degrees = s_Swerve.getYaw().getDegrees();      
      if (degrees < 0) {
        degrees += 360;
      }
      rotationVal = rController.calculate(degrees);
    }
    else {
      rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    }

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true);
  }
}