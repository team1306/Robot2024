package frc.robot.subsystems;

import static frc.robot.Constants.BACK_LEFT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.BACK_RIGHT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.FRONT_LEFT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.FRONT_RIGHT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.INCLUDE_AUTO;
import static frc.robot.Constants.INCLUDE_LIMELIGHT;
import static frc.robot.Constants.LIMELIGHT_NAME;
import static frc.robot.Constants.LOOP_TIME_SECONDS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MotorUtil;
import frc.robot.util.Utilities;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;



//Implemented as Ramsete (Differential)
public class DriveTrain extends SubsystemBase{
    //Track width in meters
    public static final double TRACK_WIDTH = 0;

    private static final String AUTO_NAME = "Path";
    
    //Percentage
    public static double MAX_SPEED = .5;

    private double currentSpeedMultipler = 1;

    private CANSparkMax leftLeader;
    private CANSparkMax leftFollower;

    private CANSparkMax rightLeader;
    private CANSparkMax rightFollower;

    private Encoder rEncoder, lEncoder;

    private DifferentialDrivePoseEstimator poseEstimator;

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private ChassisSpeeds lastSpeeds;

    public DriveTrain(){
        gyro.reset();
    
        leftLeader = MotorUtil.initSparkMax(FRONT_LEFT_DRIVE_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        leftFollower = MotorUtil.initSparkMax(BACK_LEFT_DRIVE_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        rightLeader = MotorUtil.initSparkMax(FRONT_RIGHT_DRIVE_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);
        rightFollower = MotorUtil.initSparkMax(BACK_RIGHT_DRIVE_MOTOR_ID, MotorType.kBrushless, IdleMode.kBrake);

        leftLeader.setInverted(true);
        leftFollower.follow(leftLeader, false);

        rightLeader.setInverted(false);
        rightFollower.follow(rightLeader, false);

        rEncoder = new Encoder(4, 5, false, EncodingType.k1X);
        lEncoder = new Encoder(6, 7, true, EncodingType.k1X);

        rEncoder.reset();
        lEncoder.reset();

        rEncoder.setDistancePerPulse(0.1524 * Math.PI / 2048D);
        lEncoder.setDistancePerPulse(0.1524 * Math.PI / 2048D);


        //Pathplanner configuration
        AutoBuilder.configureLTV(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentSpeeds, // Current ChassisSpeeds supplier
                this::drive, // Method that will drive the robot given ChassisSpeeds
                LOOP_TIME_SECONDS,
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                Utilities::isRedAlliance,
                this // Reference to this subsystem to set requirements
        );
        
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), lEncoder.getDistance(), rEncoder.getDistance(),
            INCLUDE_AUTO ? PathPlannerAuto.getStaringPoseFromAutoFile(AUTO_NAME) : new Pose2d());

        SmartDashboard.putNumber("Max Speed", MAX_SPEED);
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

        setSides(leftMotorOutput, rightMotorOutput);
    }

    public void drive(DifferentialDriveWheelSpeeds wheelSpeeds){
        setSides(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    public void drive(ChassisSpeeds speeds){
        lastSpeeds = speeds;

        // Convert to wheel speeds
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        drive(wheelSpeeds);
    }

    private Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }


    private void setSides(double left, double right) {
        leftLeader.setVoltage(left + (Math.signum(MathUtil.applyDeadband(left, 5e-2) * 0.0175 * 12)));
        rightLeader.setVoltage(right + (Math.signum(MathUtil.applyDeadband(right, 5e-2) * 0.0105 * 12)));   
    }

    private void resetPose(Pose2d pose){
        poseEstimator.resetPosition(gyro.getRotation2d(), new DifferentialDriveWheelPositions(0,0), pose);
    }

    private ChassisSpeeds getCurrentSpeeds(){
        return lastSpeeds;
    }

    @Override
    public void periodic() {
        poseEstimator.update(gyro.getRotation2d(), new DifferentialDriveWheelPositions(lEncoder.getDistance(), rEncoder.getDistance()));
        if (INCLUDE_LIMELIGHT) poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d(LIMELIGHT_NAME), Timer.getFPGATimestamp());
        MAX_SPEED = SmartDashboard.getNumber("Max Speed", 1);
        SmartDashboard.putNumber("Left Encoder Output", lEncoder.getRate());
        SmartDashboard.putNumber("Right Encoder Output", rEncoder.getRate());
    }

    public Command getSetSpeedMultiplierCommand(double multiplier) {
        return new Command() {
            @Override
            public void initialize() {
                currentSpeedMultipler = multiplier;
            }

            @Override
            public void end(boolean interrupted) {
                currentSpeedMultipler = 1;
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
        (Measure<Voltage> volts) -> {
        setSides(volts.in(Volts), volts.in(Volts));}, 
        log -> {
            log.motor("left")
            .voltage(m_appliedVoltage.mut_replace(leftLeader.get() * RobotController.getBatteryVoltage(), Volts))
            .linearPosition(m_distance.mut_replace(lEncoder.getDistance(), Meters))
            .linearVelocity(m_velocity.mut_replace(lEncoder.getRate(), MetersPerSecond));

            log.motor("right")
            .voltage(m_appliedVoltage.mut_replace(rightLeader.get() * RobotController.getBatteryVoltage(), Volts))
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
}
