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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;

public class SwerveDrive extends SubsystemBase {
  private final AHRS gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private NetworkTable limelight;
  private NetworkTable rpi;

  private DoubleSolenoid limelightSolenoid;

  private Field2d field;

  private CANSparkMax driveAssist;

  private PowerDistribution pdp;

  private DutyCycle distanceLaserLeft;
  private DutyCycle distanceLaserCenter;
  private DutyCycle distanceLaserRight;

  public SwerveDrive() {
    rpi = NetworkTableInstance.getDefault().getTable("rpi");
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    //limelight.getEntry("pipeline").setNumber(1);
    gyro = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
    //gyro.restoreFactoryDefaults(); //for Pigeon
    gyro.calibrate();
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
    limelightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Swerve.limelightSolenoidChannel1, Constants.Swerve.limelightSolenoidChannel2);
    driveAssist = new CANSparkMax(Constants.Swerve.driveAssistCANId, MotorType.kBrushless);
    pdp =  new PowerDistribution(1, ModuleType.kRev);

    //distanceLaserLeft = new DutyCycle(new DigitalInput(Constants.Swerve.distanceLaserLeftChannel));
    //distanceLaserCenter = new DutyCycle(new DigitalInput(Constants.Swerve.distanceLaserLeftChannel));
    //distanceLaserRight = new DutyCycle(new DigitalInput(Constants.Swerve.distanceLaserLeftChannel));
  }
  public void setLimeLED(boolean on) {
    if(!on)
      limelight.getEntry("ledMode").setNumber(1); //turn off
    else
      limelight.getEntry("ledMode").setNumber(3); //turn on
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
  public void resetSwerveModuleAngles() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }
  public void stop() {
    Translation2d translation = new Translation2d(0, 0);
    drive(translation, 0, false, false);
   // ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, new Pose2d().getRotation());
  }
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }
  public LimelightResults getLimelightResults() {
    return LimelightHelpers.getLatestResults("limelight");
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
  public double getConeAngle() {
    return rpi.getEntry("cone_angle").getDouble(0);
  }
  public double isConeFound() {
    return rpi.getEntry("cone_found").getDouble(0);
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
  public double[] getBoundingBoxX() {
    Number[] defVal = {0,0};
    double[] defaultReturn = {0,0};
    var corners = limelight.getEntry("tcornxy").getNumberArray(defVal);
    if (corners.length < 2) return defaultReturn;
    double minX = corners[0].doubleValue();
    double minY = corners[1].doubleValue();
    double maxX = corners[0].doubleValue();
    double maxY = corners[1].doubleValue();
    for (int i = 2; i< corners.length; i += 2) {
      double xCorner = corners[i].doubleValue();
      double yCorner = corners[i + 1].doubleValue();
      if (xCorner < minX) {
        minX = xCorner;
      }
      if (yCorner < minY) {
        minY = yCorner;
      }
      if (xCorner > maxX) {
        maxX = xCorner;
      }
      if (yCorner > maxY) {
        maxY = yCorner;
      }
    }
    double xMidpoint = (maxX + minX) / 2;
    double yMidpoint = (maxY + minY) / 2;
    double [] midpoint = {yMidpoint, xMidpoint};
    return midpoint;
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
        return Rotation2d.fromDegrees(- gyro.getYaw());
  }
  public float getGyroRoll() {
    return gyro.getRoll();
  }
  public LimelightTarget_Retro getBestLimelightTarget() {
    int bestResult = 0;
    double lowestX = 100;    
    LimelightTarget_Retro[] currentResult = getLimelightResults().targetingResults.targets_Retro;
    for (int i = 0; i< currentResult.length; i++) {
      if (currentResult[i].tx < lowestX) {
        lowestX = currentResult[i].tx;
        bestResult = i;
      }
    }
    if (currentResult.length == 0) {
      return null;
    }
    return currentResult[bestResult];
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
              false, 
            this // Requires this drive subsystem
         )
     );
  }
  public void limelightDown() {
    limelightSolenoid.set(Value.kReverse);
  }
  public void limelightUp() {
    limelightSolenoid.set(Value.kForward);
  }
  public double getCurrentVelocity() {
    return mSwerveMods[0].getState().speedMetersPerSecond;
  }
  public void setDriveAssistVelocity(double velocity) {
    //driveAssist.set
  }
  @Override
  public void periodic() {
    int targetToGet = 0;
    if (RobotContainer.getDriverJoystick().getRawButton(6)) {
      targetToGet = 1;
    }
    swerveOdometry.update(getYaw(), getModulePositions());
    field.setRobotPose(getPose());
    double [] boundingBox = getBoundingBoxX();
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
          "Bounding box Y", boundingBox[0]);
    SmartDashboard.putNumber(
        "Bounding box X", boundingBox[1]);
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
    SmartDashboard.putNumber(
                  "gyro pitch", gyro.getPitch());
    SmartDashboard.putNumber(
                    "Wonky swerve motor", mSwerveMods[3].getSetVelocity());
    SmartDashboard.putNumber(
                  "gyro Roll", gyro.getRoll());
                  SmartDashboard.putNumber(
                  "gyro Accel X", gyro.getRawAccelX());
                  SmartDashboard.putNumber(
                  "gyro Accel Y", gyro.getRawAccelY());
                  SmartDashboard.putNumber(
                  "gyro Accel Z", gyro.getRawAccelZ());
                  SmartDashboard.putNumber(
                    "Cone angle", getConeAngle());
                  SmartDashboard.putNumber( 
                  "limelight result x", getLimelightResults().targetingResults.targets_Retro.length);
    for (int i = 0; i < 20; i++) {
      SmartDashboard.putNumber(
          "PDP Channel " + i,pdp.getCurrent(i));
    }
  }
}