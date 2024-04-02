package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.util.DashboardGetter;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MotorUtil;
import frc.robot.util.Utilities;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import java.util.ArrayList;


//Implemented as Ramsete (Differential)
public class DriveTrain extends SubsystemBase {
    //Track width in meters
    public static final double TRACK_WIDTH = Units.inchesToMeters(25.875);
    public static double leftDowntiplier = 0.00;
    public static double rightDowntiplier = 0.0;
    // public static double leftFriction = 0;
    // public static double rightFriction = 0;

    // TODO: WHAT SHOULD THIS BE? IS THIS NEEDED?
    private static final String AUTO_NAME = "abcdef";

    private static final double leftKS = 0;//0.0087513; volts
    private static double leftKV = 3; //0.24656; volts seconds per meter
    private static double leftKA = 0.0001 * (2.9 / 0.3); // volts seconds squared per meter
    private static double leftP = 1, leftD = 0.01; // 0.14339;
    private SimpleMotorFeedforward leftFeedforward;
    private final PIDController leftPID;

    private static final double rightKS = 0; //-0.010876;
    private static double rightKV = 3; //0.24307;
    private static double rightKA = 0.0001 * (2.7/0.3); //0.080477;
    private static double rightP = .7, rightD = 0.03; //0.0032142;
    private SimpleMotorFeedforward rightFeedforward;
    private final PIDController rightPID;
    //Percentage
    public static double maxSpeed = 1;

    private double currentSpeedMultiplier = 1;

    private CANSparkMax leftLeader;
    private CANSparkMax leftFollower;
    private final Field2d m_field = new Field2d();
    private CANSparkMax rightLeader;
    private CANSparkMax rightFollower;
    
    private final ArrayList<CANSparkMax> motorControllers;
    private Encoder rEncoder, lEncoder;

    private DifferentialDrivePoseEstimator poseEstimator;

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
    public final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private DifferentialDriveWheelSpeeds lastWheelSpeeds = new DifferentialDriveWheelSpeeds();

    public void setPoseToVisionPosition() {
        resetPose(LimelightHelpers.getBotPose2d_wpiBlue(LIMELIGHT_NAME));
    }

    public DriveTrain(){
        gyro.reset();
        leftLeader = MotorUtil.initSparkMax(FRONT_LEFT_DRIVE_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        leftFollower = MotorUtil.initSparkMax(BACK_LEFT_DRIVE_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        rightLeader = MotorUtil.initSparkMax(FRONT_RIGHT_DRIVE_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        rightFollower = MotorUtil.initSparkMax(BACK_RIGHT_DRIVE_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        leftLeader.setInverted(false);
        leftFollower.follow(leftLeader, false);

        rightLeader.setInverted(true);
        rightFollower.follow(rightLeader, false);
        
        rEncoder = new Encoder(4, 5, false, EncodingType.k1X);
        rEncoder.reset();
        rEncoder.setDistancePerPulse(Units.inchesToMeters(6) * Math.PI / 2048D); // meters

        lEncoder = new Encoder(6, 7, true, EncodingType.k1X);
        lEncoder.reset();
        lEncoder.setDistancePerPulse(Units.inchesToMeters(6) * Math.PI / 2048D); // meters
        
        leftFeedforward = new SimpleMotorFeedforward(leftKS, leftKV, leftKA);
        rightFeedforward = new SimpleMotorFeedforward(rightKS, rightKV, rightKA);
        leftPID = new PIDController(leftP, 0, leftD);
        rightPID = new PIDController(rightP, 0, rightD);

        motorControllers = Utilities.listFromParams(leftLeader, rightLeader, leftFollower, rightFollower);
        
        //Pathplanner configuration
        AutoBuilder.configureLTV(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentSpeeds, // Current ChassisSpeeds supplier
                this::drive,
                .02, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                Utilities::isRedAlliance,
                this // Reference to this subsystem to set requirements
        );
        
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), lEncoder.getDistance(), rEncoder.getDistance(),
            INCLUDE_AUTO ? PathPlannerAuto.getStaringPoseFromAutoFile(AUTO_NAME) : new Pose2d());

        //SmartDashboard.putNumber("Left Drive Static Friction", 0);  
        //SmartDashboard.putNumber("Right Drive Static Friction", 0);

        DashboardGetter.addGetDoubleData("Max Speed", maxSpeed, value -> maxSpeed = value);
        DashboardGetter.addGetDoubleData("Right Drive Downtiplier", rightDowntiplier, value -> rightDowntiplier = value);
        DashboardGetter.addGetDoubleData("Left Drive Downtiplier", leftDowntiplier, value -> leftDowntiplier = value);
        DashboardGetter.addGetDoubleData("Left Drive P", leftP, value -> leftP = value);
        DashboardGetter.addGetDoubleData("Right Drive P", rightP, value -> rightP = value);
        DashboardGetter.addGetDoubleData("Left Drive D", leftD, value -> leftD = value);
        DashboardGetter.addGetDoubleData("Right Drive D", rightD, value -> rightD = value);
        DashboardGetter.addGetDoubleData("Left Drive v", leftKV, value -> leftKV = value);
        DashboardGetter.addGetDoubleData("Right Drive v", rightKV, value -> rightKV = value);
        DashboardGetter.addGetDoubleData("Left Drive a", leftKA, value -> leftKA = value);
        DashboardGetter.addGetDoubleData("Right Drive a", rightKA, value -> rightKA = value);


    }
    
    public void setSideVoltages(double left, double right) {
        double leftOutput = (left * currentSpeedMultiplier + (Math.signum(MathUtil.applyDeadband(left, 12e-2))) * 12 * 0.0175) * maxSpeed;
        double rightOutput = (right * currentSpeedMultiplier + (Math.signum(MathUtil.applyDeadband(right, 12e-2))) * 12 * 0.0105) * maxSpeed;
        
        if (DriverStation.isTeleopEnabled()) {
            leftOutput *= 1 - leftDowntiplier;
            rightOutput *= 1 - rightDowntiplier;
        }

        leftLeader.setVoltage(leftOutput);
        rightLeader.setVoltage(rightOutput);   
    }

    private void setSidePercentages(double left, double right) {
        setSideVoltages(left * 12D, right * 12D);
    }

    public void arcadeDrive(double speed, double rotation){
        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("Rotation", rotation);
        // Clamp inputs
        speed = MotorUtil.clampPercent(speed);
        rotation = MotorUtil.clampPercent(rotation);

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);
        double leftMotorOutput, rightMotorOutput;

        if (speed >= 0) {
            if (rotation >= 0) {
                // Quadrant 1 (+R, +S)
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            } else {
                // Quadrant 2 (-R, +S)
                leftMotorOutput = speed + rotation;
                rightMotorOutput = maxInput;
            }
        } else {
            if (rotation >= 0) {
                // Quadrant 4 (+R, -S)
                leftMotorOutput = speed + rotation;
                rightMotorOutput = maxInput;
            } else {
                // Quadrant 3 (-R, -S)
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rotation;
            }
        }
        if (speed == 0 && rotation == 0){
            leftMotorOutput = 0;
            rightMotorOutput = 0;
        }
        SmartDashboard.putNumber("Left Drive Output", leftMotorOutput);
        SmartDashboard.putNumber("Right Drive Output", rightMotorOutput);
        setSidePercentages(leftMotorOutput, rightMotorOutput);
    }

    public void driveMetersPerSecond(DifferentialDriveWheelSpeeds wheelSpeeds) {
        double rightVoltage = rightFeedforward.calculate(lastWheelSpeeds.rightMetersPerSecond, wheelSpeeds.rightMetersPerSecond, LOOP_TIME_SECONDS);
        rightVoltage += rightPID.calculate(rEncoder.getRate(), wheelSpeeds.rightMetersPerSecond);

        double leftVoltage = leftFeedforward.calculate(lastWheelSpeeds.leftMetersPerSecond, wheelSpeeds.leftMetersPerSecond, LOOP_TIME_SECONDS);
        leftVoltage += leftPID.calculate(lEncoder.getRate(), wheelSpeeds.leftMetersPerSecond);
        
        setSideVoltages(leftVoltage, rightVoltage);
        lastWheelSpeeds = wheelSpeeds;
    }

    public void drive(ChassisSpeeds speeds){
        // Convert to wheel speeds
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        driveMetersPerSecond(wheelSpeeds);
    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public DifferentialDriveWheelPositions getWheelPositions() {
        return new DifferentialDriveWheelPositions(lEncoder.getDistance(), rEncoder.getDistance());
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(gyro.getRotation2d(), getWheelPositions(), pose);
    }

    public void resetGyro(double angle) {
        gyro.reset();
        gyro.setAngleAdjustment(angle);
    }

    private ChassisSpeeds getCurrentSpeeds(){
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(lEncoder.getRate(), rEncoder.getRate()));
    }

    @Override
    public void periodic() {
        leftPID.setP(leftP);
        rightPID.setP(rightP);
        leftPID.setD(leftD);
        rightPID.setD(rightD);
        leftFeedforward = new SimpleMotorFeedforward(leftKS, leftKV, leftKA);
        rightFeedforward = new SimpleMotorFeedforward(rightKS, rightKV, rightKA);
        
        SmartDashboard.putNumber("gyro", getRotation().getDegrees());
        poseEstimator.update(gyro.getRotation2d(), new DifferentialDriveWheelPositions(lEncoder.getDistance(), rEncoder.getDistance()));
        m_field.setRobotPose(getPose());
        SmartDashboard.putData("Field", m_field);
        if (INCLUDE_LIMELIGHT && LimelightHelpers.getTV(LIMELIGHT_NAME)) poseEstimator.addVisionMeasurement(Utilities.getRobotPos(), Timer.getFPGATimestamp());
        SmartDashboard.putNumber("left vel drivetrain", lEncoder.getRate());
        SmartDashboard.putNumber("right vel drivetrain", rEncoder.getRate());
        SmartDashboard.putNumber("left pos", lEncoder.getDistance());
        SmartDashboard.putNumber("right pos", rEncoder.getDistance());
        SmartDashboard.putNumber("left applied out", leftLeader.getAppliedOutput());
        SmartDashboard.putNumber("right applied out", rightLeader.getAppliedOutput());

        //rightFriction = SmartDashboard.getNumber("Left Drive Static Friction", 0);  
        //leftFriction = SmartDashboard.getNumber("Right Drive Static Friction", 0);
        SmartDashboard.putNumber("left target vel drivetrain", leftPID.getSetpoint());
                SmartDashboard.putNumber("right target vel drivetrain", rightPID.getSetpoint());


    }

    public Command getSetSpeedMultiplierCommand(double multiplier) {
        return new Command() {
            @Override
            public void initialize() {
                currentSpeedMultiplier = multiplier;
            }

            @Override
            public void end(boolean interrupted) {
                currentSpeedMultiplier = 1;
            }
        };
    }

    
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
    
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(new Config(),
    new Mechanism(
        (Measure<Voltage> volts) ->
        setSideVoltages(volts.magnitude(), volts.magnitude()),
        log -> {
            log.motor("left")
            .voltage(m_appliedVoltage.mut_replace(leftLeader.getAppliedOutput(), Volts))
            .linearPosition(m_distance.mut_replace(lEncoder.getDistance(), Meters))
            .linearVelocity(m_velocity.mut_replace(lEncoder.getRate(), MetersPerSecond));

            log.motor("right")
            .voltage(m_appliedVoltage.mut_replace(rightLeader.getAppliedOutput(), Volts))
            .linearPosition(m_distance.mut_replace(rEncoder.getDistance(), Meters))
            .linearVelocity(m_velocity.mut_replace(rEncoder.getRate(), MetersPerSecond));
        }, this));

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
    /**
     * returns a command to drive by setpoint percentages
     * @param leftSpeed speed of left wheel
     * @param rightSpeed speed of right wheel
     * @return generated command
     */
    public Command driveBySetpointPercentagesCommand(double leftSpeed, double rightSpeed) {
        return new Command(){
            {
                addRequirements(DriveTrain.this);
            }

            @Override
            public void execute() {
                setSidePercentages(leftSpeed, rightSpeed);
            }

            @Override
            public void end(boolean interrupted) {
                setSidePercentages(0, 0);
            }
        };
    }
    
    
  public void pushCurrentLimitToAllDrivetrainMotors(int amps) {
    motorControllers.forEach(motor -> motor.setSmartCurrentLimit(amps));
  }
}
