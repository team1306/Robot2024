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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MotorUtil;
import frc.robot.util.Utilities;


//Implemented as Ramsete (Differential)
public class DriveTrain extends SubsystemBase{
    //Track width in meters
    public static final double TRACK_WIDTH = 0;
    //Above 1
    public static double leftMultiplier = 0;
    public static double rightMulitplier = 0;
    // public static double leftFriction = 0;
    // public static double rightFriction = 0;

    private static final String AUTO_NAME = "Close Rings from Start-Mid";
    
    //Percentage
    public static double MAX_SPEED = 1;

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

        rEncoder = new Encoder(4, 5, true, EncodingType.k1X);
        rEncoder.reset();
        rEncoder.setDistancePerPulse(360/2048); // DEGREES_PER_REVOLUTION / CYCLES PER REVOLUTION

        lEncoder = new Encoder(6, 7, true, EncodingType.k1X);;
        lEncoder.reset();
        lEncoder.setDistancePerPulse(360/2048); // DEGREES_PER_REVOLUTION / CYCLES PER REVOLUTION
        
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
        SmartDashboard.putNumber("Left Drive Multiplier", 0);  
        SmartDashboard.putNumber("Right Drive Multiplier", 0); 
        //SmartDashboard.putNumber("Left Drive Static Friction", 0);  
        //SmartDashboard.putNumber("Right Drive Static Friction", 0); 
     }
    
    private void setSides(double left, double right) {
        leftLeader.set(left * currentSpeedMultipler * MAX_SPEED * (1 + leftMultiplier) + (Math.signum(MathUtil.applyDeadband(left, 5e-2) * 0.0175)));
        rightLeader.set(right * currentSpeedMultipler * MAX_SPEED * (1 + rightMulitplier) + (Math.signum(MathUtil.applyDeadband(right, 5e-2) * 0.0105)));   
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

    private void resetPose(Pose2d pose){
        poseEstimator.resetPosition(gyro.getRotation2d(), new DifferentialDriveWheelPositions(0,0), pose);
    }

    private ChassisSpeeds getCurrentSpeeds(){
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(lEncoder.getRate(), rEncoder.getRate()));
    }

    @Override
    public void periodic() {
        poseEstimator.update(gyro.getRotation2d(), new DifferentialDriveWheelPositions(lEncoder.getDistance(), rEncoder.getDistance()));
        if (INCLUDE_LIMELIGHT) poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d(LIMELIGHT_NAME), Timer.getFPGATimestamp());
        MAX_SPEED = SmartDashboard.getNumber("Max Speed", 1);
        SmartDashboard.putNumber("Left Encoder Output", lEncoder.getRate());
        SmartDashboard.putNumber("Right Encoder Output", rEncoder.getRate());

        rightMulitplier = SmartDashboard.getNumber("Right Drive Multiplier", 0);
        leftMultiplier = SmartDashboard.getNumber("Left Drive Multiplier", 0);

        //rightFriction = SmartDashboard.getNumber("Left Drive Static Friction", 0);  
        //leftFriction = SmartDashboard.getNumber("Right Drive Static Friction", 0); 
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
}
