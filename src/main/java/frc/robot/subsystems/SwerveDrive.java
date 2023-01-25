package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveDrive extends SubsystemBase {
  private final AHRS gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private NetworkTable limelight;

  private Field2d field;

  public SwerveDrive() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    limelight.getEntry("pipeline").setNumber(1);
    gyro = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
    //gyro.restoreFactoryDefaults(); //for Pigeon
    zeroGyro();
    

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }
  public void alternateDrive(double xSpeed, double ySpeed, double omegaSpeed, Pose2d robotPose2d) {
    ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation());
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }
  public void stop() {
    ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, new Pose2d().getRotation());
  }
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public double getLimelightX() {
    return limelight.getEntry("tx").getDouble(0);
  }
  public double getLimelightObjectSize() {
    return limelight.getEntry("ta").getDouble(0);
  }

  public double getLimelightY() {
    return limelight.getEntry("ty").getDouble(0);
  }
  public void setLimelightPipeline(int lineNum) {
    limelight.getEntry("pipeline").setNumber(lineNum);
  }
  public double limelightHasTarget() {
    return limelight.getEntry("tv").getDouble(0);
  }
  public void setLimelightLED(boolean state) {
    if(state == false)
      limelight.getEntry("ledMode").setNumber(1);
    else {
      limelight.getEntry("ledMode").setNumber(3);
    }
  
  }
//   public SwerveModuleState[] getStates() { //TODO this can probably be removed, the getmodulepositions seems to replace it
//     SwerveModuleState[] states = new SwerveModuleState[4];
//     for (SwerveModule mod : mSwerveMods) {
//       states[mod.moduleNumber] = mod.getState();
//     }
//     return states;
//   }

  public void zeroGyro() {
    gyro.zeroYaw();
  }

  public SwerveModulePosition[] getModulePositions(){ //TODO this is new, might need to double check
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}

  public Rotation2d getYaw() {
    // return (Constants.Swerve.invertGyro)
    //     ? Rotation2d.fromDegrees(360 - gyro.getYaw())
    //     : Rotation2d.fromDegrees(gyro.getYaw());

    if (gyro.isMagnetometerCalibrated()) {
        // We will only get valid fused headings if the magnetometer is calibrated
        return Rotation2d.fromDegrees(gyro.getFusedHeading());
        }
    //
    //    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
  }
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
         new InstantCommand(() -> {
           // Reset odometry for the first path you run during auto
           if(isFirstPath){
               this.resetOdometry(traj.getInitialHolonomicPose());
           }
         }),
         new PPSwerveControllerCommand(
             traj, 
             this::getPose, // Pose supplier
             Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
             new PIDController(0.1, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             new PIDController(0.1, 0, 0), // Y controller (usually the same values as X controller)
             new PIDController(0.1, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             this::setModuleStates, // Module states consumer
             this // Requires this drive subsystem
         )
     );
 }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber(
          "Drive Enc " + mod.moduleNumber, mod.getDriveEncoderPosition());
    }
    SmartDashboard.putNumber(
          "Gyro", gyro.getAngle());
    SmartDashboard.putNumber(
            "2dPose X", getPose().getX());
    SmartDashboard.putNumber(
              "2dPose Y", getPose().getY());
    SmartDashboard.putNumber(
                "Limelight X", getLimelightX());
    SmartDashboard.putNumber(
                "Limelight Y", getLimelightY());
    SmartDashboard.putNumber(
                "Limelight obj size", getLimelightObjectSize());
    SmartDashboard.putNumber(
                  "Swerve Yaw", getYaw().getDegrees());
  }
}