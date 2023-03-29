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
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SerialPort.FlowControl;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
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
  //private DutyCycle distanceLaserRight;
  private double rightLaserDistance;
  int loopcount = 0;
  int cstate = 0;

  private Relay headlights;

  private static final double LimelightXFov = 72; //59.6
	private static final int LimelightXResolution = 320;

	private static final double ConeSideLength = 13.776;
	private static final double ConeBaseLength = 8.375;
	private static final double ConeEdgeAngleOffset = 21.56;
	private static final double ConeBaseAngleOffset = 90;

  private SerialPort serPort = new SerialPort(19200, Port.kMXP);
  private static double boundingBoxWidth = 0;
  private static double lowestLimelightTargetY = 0;
  public SwerveDrive() {
    rpi = NetworkTableInstance.getDefault().getTable("rpi");
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    //limelight.getEntry("pipeline").setNumber(1);
    gyro = new AHRS(SerialPort.Port.kUSB); // NavX connected over MXP //, (byte) 200
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

    headlights = new Relay(Constants.Swerve.headlightsRelayChannel);

    distanceLaserLeft = new DutyCycle(new DigitalInput(Constants.Swerve.distanceLaserLeftChannel));
    distanceLaserCenter = new DutyCycle(new DigitalInput(Constants.Swerve.distanceLaserCenterChannel));
    //distanceLaserRight = new DutyCycle(new DigitalInput(Constants.Swerve.distanceLaserLeftChannel));
    serPort.setFlowControl(FlowControl.kNone);
    serPort.setReadBufferSize(128);
    serPort.flush();
    rightLaserDistance = 0;
    lowestLimelightTargetY = 0;
  }
  public void setLimeLED(boolean on) {
    if(!on)
      limelight.getEntry("ledMode").setNumber(1); //turn off
    else
      limelight.getEntry("ledMode").setNumber(3); //turn on
  }
  public void setHeadlights(boolean on) {
    if (!on) {
      headlights.set(Relay.Value.kOff);
    }
    else {
      headlights.set(Relay.Value.kForward);
    }
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
  public void setToXOrientation() {    
    mSwerveMods[0].setDesiredState(new SwerveModuleState(0.05, Rotation2d.fromDegrees(45)), true);
    mSwerveMods[1].setDesiredState(new SwerveModuleState(0.05, Rotation2d.fromDegrees(-45)), true);
    mSwerveMods[2].setDesiredState(new SwerveModuleState(0.05, Rotation2d.fromDegrees(-45)), true);
    mSwerveMods[3].setDesiredState(new SwerveModuleState(0.05, Rotation2d.fromDegrees(45)), true);
  }
  public void resetSwerveModuleAngles() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
      mod.setDesiredState(new SwerveModuleState(0.05, Rotation2d.fromDegrees(0)), true);
    }
  }
  public void setModulesStraight() {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(new SwerveModuleState(0.05, Rotation2d.fromDegrees(0)), true);
    }
  }
  public double getLeftLaserValue() {
    return distanceLaserLeft.getOutput() * 180 + 20;
  }
  public double getCenterLaserValue() {
    return distanceLaserCenter.getOutput() * 180 + 20;
  }
  public double getRightLaserValue() {
    return rightLaserDistance;
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
  public int[] getBoundingBoxMinMaxX() {
    Number[] defVal = {0,0};
    int[] defaultReturn = {0,0};
    var corners = limelight.getEntry("tcornxy").getNumberArray(defVal);
    if (corners.length < 2) return defaultReturn;
    int minX = corners[0].intValue();
    int maxX = corners[0].intValue();
    for (int i = 2; i< corners.length; i += 2) {
      int xCorner = corners[i].intValue();      
      if (xCorner < minX) {
        minX = xCorner;
      }      
      if (xCorner > maxX) {
        maxX = xCorner;
      }     
    }    
    
    int[] minMaxX = {minX, maxX};
    return minMaxX;
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
    boundingBoxWidth = maxX - minX;
    lowestLimelightTargetY = maxY; //largest y value is lowest on the image
    return midpoint;
  }
  public double getBoundingBoxWidth() {
    return boundingBoxWidth;
  }
  public double getlimelightLowestYValue() {
    return lowestLimelightTargetY;
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

    // if (gyro.isMagnetometerCalibrated()) {
    //     // We will only get valid fused headings if the magnetometer is calibrated
    //     return Rotation2d.fromDegrees(gyro.getFusedHeading());
    //     }
    //
    //    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        double gyroYaw = -gyro.getYaw() + 180;
        if (gyroYaw > 180) {
          gyroYaw -= 360;
        }
        else if (gyroYaw < -180) {
          gyroYaw += 360;
        }
        return Rotation2d.fromDegrees(gyroYaw);
  }
  public float getGyroRoll() {
    return gyro.getRoll();
  }
  public float getGyroPitch() {
    return gyro.getPitch();
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
               //this.zeroGyro();
           }
         }),
         new PPSwerveControllerCommand(
             traj, 
             this::getPose, // Pose supplier
             Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
             new PIDController(4, 0, 0.3), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
             new PIDController(4, 0, 0.3), // Y controller (usually the same values as X controller)
             new PIDController(3.2, 0, 0.3), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
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
  public static class Point {

    private final double X;
    private final double Y;
  
    public double GetX() { return X; }
    public double GetY() { return Y; }
  
    public Point(double x, double y) {
      X = x;
      Y = y;
    }
  
    public Point CopyAndOffset(Point offset) {
      return new Point(X + offset.X, Y + offset.Y);
    }
  
    public Point CopyAndRotate(Angle rotation) {
  
      final double newX = X * rotation.Cosine() + Y * rotation.Sine();
      final double newY = -X * rotation.Sine() + Y * rotation.Cosine();
      return new Point(newX, newY);
    }
  
  }
  public enum OrientationCase {
		TipDirectlyAway,
		TipDirectlyTowards,
		TipLeftAndAway,
		TipLeftAndTowards,
		TipRightAndAway,
		TipRightAndTowards,
		NoMatch
	}
  static class Angle {

    private final double Value;
  
    public Angle(double value) {
  
      while (value > 180) {
        value -= 360;
      }
  
      while (value < -180) {
        value += 360;
      }
  
      Value = value;
    }
  
    public Angle CopyWithOffset(double offset) {
  
      return new Angle(Value + offset);
    }
  
    public Angle CopyInverted() {
      return CopyWithOffset(180);
    }
  
    public boolean IsBetween(Angle antiClockwiseLimit, Angle clockwiseLimit) {
  
      // both on right half
      if (antiClockwiseLimit.Value >= 0 && clockwiseLimit.Value >= 0 && antiClockwiseLimit.Value <= clockwiseLimit.Value) {
        return Value >= antiClockwiseLimit.Value && Value <= clockwiseLimit.Value;
  
      // both on left half
      } else if (antiClockwiseLimit.Value <= 0 && clockwiseLimit.Value <= 0 && antiClockwiseLimit.Value <= clockwiseLimit.Value) {
        return Value >= antiClockwiseLimit.Value && Value <= clockwiseLimit.Value;
  
      // acw on left, cw on right
      } else if (antiClockwiseLimit.Value <= 0 && clockwiseLimit.Value >= 0) {
        return Value >= antiClockwiseLimit.Value && Value <= clockwiseLimit.Value;
  
      // both on right, inverted
      } else if (antiClockwiseLimit.Value >= 0 && clockwiseLimit.Value >= 0 && antiClockwiseLimit.Value >= clockwiseLimit.Value) {
        return (Value <= clockwiseLimit.Value && Value >= -180) || (Value > antiClockwiseLimit.Value && Value <= 180);
  
      // both on left, inverted
      } else if (antiClockwiseLimit.Value <= 0 && clockwiseLimit.Value <= 0 && antiClockwiseLimit.Value >= clockwiseLimit.Value) {
        return (Value >= antiClockwiseLimit.Value && Value <= 180) || (Value <= clockwiseLimit.Value && Value >= -180);
  
      // acw of right, cw on left
      } else if (antiClockwiseLimit.Value >= 0 && clockwiseLimit.Value <= 0) {
        return (Value <= clockwiseLimit.Value && Value >= -180) || (Value >= antiClockwiseLimit.Value && Value <= 180);
      }
  
      System.out.println("IsBetweenInclusive() wasn't able to match the parameters to a case it knows how to evaluate. angle=" 
      + Value + ", counterClockwiseLimit=" + antiClockwiseLimit.Value + ", clockwiseLimit=" + clockwiseLimit.Value);
  
      return false;
    }
  
    public boolean ClockwiseOf(Angle angle) {
  
      return IsBetween(angle, angle.CopyWithOffset(180));
    }
  
    public boolean AntiClockwiseOf(Angle angle) {
  
      return IsBetween(angle.CopyWithOffset(180), angle);
    }
  
    public double Sine() {
  
      return Math.sin(Value * Math.PI / 180);
    }
    
    public double Cosine() {
  
      return Math.cos(Value * Math.PI / 180);
    }
  
    public static Angle Between(Angle antiClockwiseAngle, Angle clockwiseAngle) {
      
      // both on right half
      if (antiClockwiseAngle.Value >= 0 && clockwiseAngle.Value >= 0 && antiClockwiseAngle.Value <= clockwiseAngle.Value) {
        return new Angle(clockwiseAngle.Value - antiClockwiseAngle.Value);
  
      // both on left half
      } else if (antiClockwiseAngle.Value <= 0 && clockwiseAngle.Value <= 0 && antiClockwiseAngle.Value <= clockwiseAngle.Value) {
        return new Angle(clockwiseAngle.Value - antiClockwiseAngle.Value);
  
      // ccw on left, cw on right
      } else if (antiClockwiseAngle.Value <= 0 && clockwiseAngle.Value >= 0) {
        return new Angle(clockwiseAngle.Value - antiClockwiseAngle.Value);
  
      // both on right, inverted
      } else if (antiClockwiseAngle.Value >= 0 && clockwiseAngle.Value >= 0 && antiClockwiseAngle.Value >= clockwiseAngle.Value) {
        return new Angle(360 - (antiClockwiseAngle.Value - clockwiseAngle.Value));
  
      // both on left, inverted
      } else if (antiClockwiseAngle.Value <= 0 && clockwiseAngle.Value <= 0 && antiClockwiseAngle.Value >= clockwiseAngle.Value) {
        return new Angle(360 - (antiClockwiseAngle.Value - clockwiseAngle.Value));
  
      // ccw of right, cw on left
      } else if (antiClockwiseAngle.Value >= 0 && clockwiseAngle.Value <= 0) {
        return new Angle(360 - (antiClockwiseAngle.Value - clockwiseAngle.Value));
      }
  
      System.out.println("AngleBetween() wasn't able to match the parameters to a case it knows how to evaluate. antiClockwiseAngle=" 
      + antiClockwiseAngle + ", clockwiseAngle=" + clockwiseAngle.Value);
  
      return new Angle(0);
    }
  
    public String ToString() {
      return "Angle { Value = " + Value + "}";
    }

  }

	static private Angle GetInCameraAngle(final double xPosition, final int xResolution, final double xFov) {

		final double normalizedXPosition = (xPosition - xResolution / 2.0) / (xResolution / 2.0);

		return new Angle(normalizedXPosition * xFov / 2);
	}

	static private OrientationCase GetOrientationCase(final int boundingBoxLeftEdge, final int boundingBoxRightEdge, final Angle coneAngle) {

		final Angle angleToLeftEdge = GetInCameraAngle(boundingBoxLeftEdge, LimelightXResolution, LimelightXFov);
		final Angle angleToRightEdge = GetInCameraAngle(boundingBoxRightEdge, LimelightXResolution, LimelightXFov);
		final Angle angleToCenter = GetInCameraAngle((boundingBoxLeftEdge + boundingBoxRightEdge) / 2.0, LimelightXResolution, LimelightXFov);

		final Angle coneLeftSideAngleFromBase = coneAngle.CopyWithOffset(ConeEdgeAngleOffset);
		final Angle coneRightSideAngleFromBase = coneAngle.CopyWithOffset(-ConeEdgeAngleOffset);
		final Angle coneLeftSideAngleFromTip = coneLeftSideAngleFromBase.CopyWithOffset(180);
		final Angle coneRightSideAngleFromTip = coneRightSideAngleFromBase.CopyWithOffset(180);
		final Angle coneBaseAngleFromLeftCorner = coneAngle.CopyWithOffset(ConeBaseAngleOffset);
		final Angle coneBaseAngleFromRightCorner = coneAngle.CopyWithOffset(-ConeBaseAngleOffset);

		if (coneLeftSideAngleFromBase.ClockwiseOf(angleToLeftEdge) && coneRightSideAngleFromBase.AntiClockwiseOf(angleToRightEdge)) {
			return OrientationCase.TipDirectlyAway;
		}

		if (coneRightSideAngleFromTip.AntiClockwiseOf(angleToLeftEdge) && coneLeftSideAngleFromTip.ClockwiseOf(angleToRightEdge)) {
			return OrientationCase.TipDirectlyTowards;
		}

		if (coneAngle.AntiClockwiseOf(angleToCenter) && coneBaseAngleFromLeftCorner.ClockwiseOf(angleToRightEdge)) {
			return OrientationCase.TipLeftAndAway;
		}

		if (coneAngle.AntiClockwiseOf(angleToCenter) && coneBaseAngleFromLeftCorner.AntiClockwiseOf(angleToRightEdge)) {
			return OrientationCase.TipLeftAndTowards;
		}

		if (coneAngle.ClockwiseOf(angleToCenter) && coneBaseAngleFromRightCorner.AntiClockwiseOf(angleToLeftEdge)) {
			return OrientationCase.TipRightAndAway;
		}

		if (coneAngle.ClockwiseOf(angleToCenter) && coneBaseAngleFromRightCorner.ClockwiseOf(angleToLeftEdge)) {
			return OrientationCase.TipRightAndTowards;
		}

		System.out.println("GetOrinetationCase() wasn't able to match the parameters to a case it knows how to evaluate. boundingBoxLeftEdge=" +
		boundingBoxLeftEdge + ", boundingBoxRightEdge=" + boundingBoxRightEdge + ", coneAngle=" + coneAngle);

		return OrientationCase.NoMatch;
	}

	static public Point GetConePosition(final int boundingBoxLeftEdge, final int boundingBoxRightEdge, final double coneAngleValue) {

    final Angle coneAngle = new Angle(coneAngleValue);

		final OrientationCase orientationCase = GetOrientationCase(boundingBoxLeftEdge, boundingBoxRightEdge, coneAngle);

    SmartDashboard.putString("CONE_0orientationCase", orientationCase.toString());

    SmartDashboard.putNumber("CONE_0boundingBoxLeftEdge", boundingBoxLeftEdge);
    SmartDashboard.putNumber("CONE_0boundingBoxRightEdge", boundingBoxRightEdge);

		final Angle angleToLeftEdge = GetInCameraAngle(boundingBoxLeftEdge, LimelightXResolution, LimelightXFov);
		final Angle angleToRightEdge = GetInCameraAngle(boundingBoxRightEdge, LimelightXResolution, LimelightXFov);

    SmartDashboard.putString("CONE_0angleToLeftEdge", angleToLeftEdge.ToString());
    SmartDashboard.putString("CONE_0angleToRightEdge", angleToRightEdge.ToString());

		final Angle coneLeftSideAngleFromBase = coneAngle.CopyWithOffset(ConeEdgeAngleOffset);
		final Angle coneRightSideAngleFromBase = coneAngle.CopyWithOffset(-ConeEdgeAngleOffset);
		final Angle coneLeftSideAngleFromTip = coneLeftSideAngleFromBase.CopyInverted();
		final Angle coneRightSideAngleFromTip = coneRightSideAngleFromBase.CopyInverted();
		final Angle coneBaseAngleFromLeftCorner = coneAngle.CopyWithOffset(ConeBaseAngleOffset);
		final Angle coneBaseAngleFromRightCorner = coneAngle.CopyWithOffset(-ConeBaseAngleOffset);

		Angle interiorCameraAngle = Angle.Between(angleToLeftEdge, angleToRightEdge);
		Angle interiorLeftAngle;
		Angle interiorRightAngle;
		double sideOppositeCameraLength;

		switch (orientationCase) {

			case TipDirectlyAway:
				interiorLeftAngle = Angle.Between(coneBaseAngleFromLeftCorner, angleToLeftEdge.CopyInverted());
				interiorRightAngle = Angle.Between(angleToRightEdge.CopyInverted(), coneBaseAngleFromRightCorner);
				sideOppositeCameraLength = ConeBaseLength;
				break;

			case TipDirectlyTowards:
				interiorLeftAngle = Angle.Between(coneBaseAngleFromRightCorner, angleToLeftEdge.CopyInverted());
				interiorRightAngle = Angle.Between(angleToRightEdge.CopyInverted(), coneBaseAngleFromLeftCorner);
				sideOppositeCameraLength = ConeBaseLength;
				break;

			case TipLeftAndAway:
				interiorLeftAngle = Angle.Between(coneRightSideAngleFromTip, angleToLeftEdge.CopyInverted());
				interiorRightAngle = Angle.Between(angleToRightEdge.CopyInverted(), coneRightSideAngleFromBase);
				sideOppositeCameraLength = ConeSideLength;
				break;

			case TipLeftAndTowards:
				interiorLeftAngle = Angle.Between(coneLeftSideAngleFromTip, angleToLeftEdge.CopyInverted());
				interiorRightAngle = Angle.Between(angleToRightEdge.CopyInverted(), coneLeftSideAngleFromBase);
				sideOppositeCameraLength = ConeSideLength;
				break;

			case TipRightAndAway:
				interiorLeftAngle = Angle.Between(coneLeftSideAngleFromBase, angleToLeftEdge.CopyInverted());
				interiorRightAngle = Angle.Between(angleToRightEdge.CopyInverted(), coneLeftSideAngleFromTip);
				sideOppositeCameraLength = ConeSideLength;
				break;

			case TipRightAndTowards:
				interiorLeftAngle = Angle.Between(coneRightSideAngleFromBase, angleToLeftEdge.CopyInverted());
				interiorRightAngle = Angle.Between(angleToRightEdge.CopyInverted(), coneRightSideAngleFromTip);
				sideOppositeCameraLength = ConeSideLength;
				break;

			default:
				System.out.println("No orientation case matched in GetDistanceToCone().");
				return new Point(-1, -1);
		}

    SmartDashboard.putString("CONE_1interiorCameraAngle", interiorCameraAngle.ToString());
    SmartDashboard.putString("CONE_2interiorLeftAngle", interiorLeftAngle.ToString());
    SmartDashboard.putString("CONE_3interiorRightAngle", interiorRightAngle.ToString());

		final double leftSideLength = sideOppositeCameraLength / interiorCameraAngle.Sine() * interiorRightAngle.Sine();
		final double rightSideLength = sideOppositeCameraLength / interiorCameraAngle.Sine() * interiorLeftAngle.Sine();

    SmartDashboard.putNumber("CONE_4leftSideLength", leftSideLength);
    SmartDashboard.putNumber("CONE_5rightSideLength", rightSideLength);

		final Point leftPoint = new Point(angleToLeftEdge.Sine() * leftSideLength, angleToLeftEdge.Cosine() * leftSideLength);
		final Point rightPoint = new Point(angleToRightEdge.Sine() * rightSideLength, angleToRightEdge.Cosine() * rightSideLength);

    SmartDashboard.putNumber("CONE_6leftPointX", leftPoint.GetX());
    SmartDashboard.putNumber("CONE_7leftPointY", leftPoint.GetY());
    SmartDashboard.putNumber("CONE_8rightPointX", rightPoint.GetX());
    SmartDashboard.putNumber("CONE_9rightPointY", rightPoint.GetY());

		Point pointToOffsetFrom;
		Point offset;
		Angle offsetRotation;

		switch (orientationCase) {

			case TipDirectlyAway:
				pointToOffsetFrom = leftPoint;
				offset = new Point(ConeBaseLength / 2, ConeBaseLength / 2);
				offsetRotation = coneAngle;
				break;

			case TipDirectlyTowards:
				pointToOffsetFrom = leftPoint;
				offset = new Point(ConeBaseLength / 2, -ConeBaseLength / 2);
				offsetRotation = coneAngle.CopyInverted();
				break;

			case TipLeftAndAway:
				pointToOffsetFrom = rightPoint;
				offset = new Point(-ConeBaseLength / 2, -ConeBaseLength / 2);
				offsetRotation = coneBaseAngleFromLeftCorner;
				break;

			case TipLeftAndTowards:
				pointToOffsetFrom = rightPoint;
				offset = new Point(-ConeBaseLength / 2, ConeBaseLength / 2);
				offsetRotation = coneBaseAngleFromLeftCorner;
				break;

			case TipRightAndAway:
				pointToOffsetFrom = leftPoint;
				offset = new Point(ConeBaseLength / 2, -ConeBaseLength / 2);
				offsetRotation = coneBaseAngleFromRightCorner;
				break;

			case TipRightAndTowards:
				pointToOffsetFrom = leftPoint;
				offset = new Point(ConeBaseLength / 2, ConeBaseLength / 2);
				offsetRotation = coneBaseAngleFromRightCorner;
				break;

			default:
				System.out.println("no orientation case matched in GetDistanceToCone().");
				return new Point(-1, -1);
		}

    SmartDashboard.putNumber("aCONE_aoffsetPointX", offset.CopyAndRotate(offsetRotation).GetX());
    SmartDashboard.putNumber("aCONE_aoffsetPointY", offset.CopyAndRotate(offsetRotation).GetY());

		final Point centerFromLimeLight = pointToOffsetFrom.CopyAndOffset(offset.CopyAndRotate(offsetRotation));

    final Point centerFromFront = centerFromLimeLight.CopyAndOffset(new Point(0, -2.5));
    Point orientationBasedOffset;

    switch (orientationCase) {
      case TipDirectlyAway: orientationBasedOffset = new Point(0, 1); break;
      case TipDirectlyTowards: orientationBasedOffset = new Point(0, 3.25); break;
      case TipLeftAndAway: orientationBasedOffset = new Point(0, 0.5); break;
      case TipLeftAndTowards: orientationBasedOffset = new Point(0, 0.75); break;
      case TipRightAndAway: orientationBasedOffset = new Point(0, 1.25); break;
      case TipRightAndTowards: orientationBasedOffset = new Point(0, 0); break;
      default: orientationBasedOffset = new Point(0, 0);
    }

    final Point center = centerFromFront.CopyAndOffset(orientationBasedOffset);

    SmartDashboard.putNumber("CONE_bcenterX",center.GetX());
    SmartDashboard.putNumber("CONE_bcenterY", center.GetY());

		return center;
	}
  @Override
  public void periodic() {
    loopcount = 0;
    int bytes_available = serPort.getBytesReceived();
    var bytes_to_read = 0;
    int chktmp = 0;
    int chksum = 0;
    long dist = 0;
    int signal = 0;
    int temperature = 0;
    while((loopcount < 2)&&(bytes_available > 0)) {
		  loopcount++;
		  if (bytes_available > 128) {
        bytes_to_read = 128;
      }
      else {
        bytes_to_read = bytes_available;
      }
      if (bytes_to_read > 0) {
        var rxbuf = serPort.read(bytes_available);
        
        
			  for(int x = 0;x<rxbuf.length;x++) { // process each character in the serial port buffer.
          int ch = rxbuf[x]; // get the character
          chktmp += ch;
          // printf("ch=%02x, cstate=%d, dcnt=%d\n",ch,cstate,dcnt);
          //if (cstate < 4)
          //if (chcnt < 16)
          //	{
          //	printf("%02X ",ch);
          //	chcnt++;
          //	}
          switch(cstate) {
            case 0: // expecting a packet start character
              if (ch == 0x59) {
                chktmp = 0x59;
                cstate++; // go to next state if we get a valid packet start character
              }
              break;
            case 1:
              if (ch == 0x59) {
                cstate++;
              }
              else {
                cstate = 0;
              }
            break;
            case 2:
              dist = ch & 0xFF;
              cstate++;
            break;
            case 3:
              dist += ((ch & 0xFF) << 8);
              SmartDashboard.putNumber("dist", dist);
              cstate++;
            break;
            case 4:
              signal = ch;
              cstate++;
            break;
            case 5:
              signal += (ch << 8);
              cstate++;
            break;
            case 6:
              temperature = ch;
              cstate++;
            break;
            case 7:
              temperature += (ch << 8);
              cstate++;
              chksum = chktmp & 0xFF;
            break;
            case 8:
              if (chksum == ch) {
                // if (dist > 400) {
                //   dist -= 415;
                // }
                if (dist >= 200 || dist == 0) {
                  dist = 200;
                }
                else if (dist < 0) {
                  dist = 0;
                }
                rightLaserDistance = dist;
              }
              cstate = 0;
            break;
          }
        }
      }
      bytes_available = serPort.getBytesReceived();
    }
    int targetToGet = 0;
    // if (RobotContainer.getDriverJoystick().getRawButton(6)) {
    //   targetToGet = 1;
    // }
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
                    "Left Laser Distance", getLeftLaserValue());
    SmartDashboard.putNumber(
                    "Center Laser Distance", getCenterLaserValue());
    SmartDashboard.putNumber(
                    "Right Laser Distance", getRightLaserValue());
    SmartDashboard.putNumber("Bounding box width", getBoundingBoxWidth());
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
    // for (int i = 0; i < 20; i++) {
    //   SmartDashboard.putNumber(
    //       "PDP Channel " + i,pdp.getCurrent(i));
    // }
  }
}