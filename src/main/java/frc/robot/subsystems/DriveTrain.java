package frc.robot.subsystems;

import static frc.robot.Constants.BACK_LEFT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.BACK_RIGHT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.FRONT_LEFT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.FRONT_RIGHT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.INCLUDE_AUTO;
import static frc.robot.Constants.INCLUDE_LIMELIGHT;
import static frc.robot.Constants.LIMELIGHT_NAME;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.vision.SwitchableDriverCam;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.MotorUtil;
import frc.robot.util.Utilities;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.MutableMeasure.mutable;


//Implemented as Ramsete (Differential)
public class DriveTrain extends SubsystemBase {
    //Track width in meters
    public static final double TRACK_WIDTH = Units.inchesToMeters(25.875);
    //Above 1
    public static double leftDowntiplier = 0.12;
    public static double rightDowntiplier = 0;
    // public static double leftFriction = 0;
    // public static double rightFriction = 0;

    // TODO: WHAT SHOULD THIS BE? IS THIS NEEDED?
    private static final String AUTO_NAME = "testPath";

    private static final double leftKS = 0.0088193;//0.0087513; volts
    private static final double leftKV = 0.27474; //0.24656; volts seconds per meter
    private static final double leftKA = 0.077361; // volts seconds squared per meter
    private static final double leftP = 0.11574; // 0.14339;
    private final SimpleMotorFeedforward leftFeedforward;
    private final PIDController leftPID;

    private static final double rightKS = 0.0056672; //-0.010876;
    private static final double rightKV = 0.19594; //0.24307;
    private static final double rightKA = 0.056995; //0.080477;
    private static final double rightP = 0.011096; //0.0032142;
    private final SimpleMotorFeedforward rightFeedforward;
    private final PIDController rightPID;
    
    //Percentage
    public static double MAX_SPEED = 1;

    private double currentSpeedMultipler = 1;

    private CANSparkMax leftLeader;
    private CANSparkMax leftFollower;
    private final Field2d m_field = new Field2d();
    private CANSparkMax rightLeader;
    private CANSparkMax rightFollower;

    private Encoder rEncoder, lEncoder;

    private DifferentialDrivePoseEstimator poseEstimator;

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwitchableDriverCam switchableDriverCam;

    private DifferentialDriveWheelSpeeds lastDriveVoltages = new DifferentialDriveWheelSpeeds();

    public DriveTrain(SwitchableDriverCam switchableDriverCam){
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
        rEncoder.reset();
        rEncoder.setDistancePerPulse(Units.inchesToMeters(6) * Math.PI / 2048D); // DEGREES_PER_REVOLUTION / CYCLES PER REVOLUTION

        lEncoder = new Encoder(6, 7, true, EncodingType.k1X);;
        lEncoder.reset();
        lEncoder.setDistancePerPulse(Units.inchesToMeters(6) * Math.PI / 2048D); // DEGREES_PER_REVOLUTION / CYCLES PER REVOLUTION
        
        leftFeedforward = new SimpleMotorFeedforward(leftKS, leftKV, leftKA);
        rightFeedforward = new SimpleMotorFeedforward(rightKS, rightKV, rightKA);
        leftPID = new PIDController(leftP, 0, 0);
        rightPID = new PIDController(rightP, 0, 0);
        
        //Pathplanner configuration
        AutoBuilder.configureRamsete(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentSpeeds, // Current ChassisSpeeds supplier
                this::drive, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                Utilities::isRedAlliance,
                this // Reference to this subsystem to set requirements
        );
        
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getRotation2d(), lEncoder.getDistance(), rEncoder.getDistance(),
            INCLUDE_AUTO ? PathPlannerAuto.getStaringPoseFromAutoFile(AUTO_NAME) : new Pose2d());

        SmartDashboard.putNumber("Max Speed", MAX_SPEED);
        //SmartDashboard.putNumber("Left Drive Static Friction", 0);  
        //SmartDashboard.putNumber("Right Drive Static Friction", 0);
        this.switchableDriverCam = switchableDriverCam;      
        SmartDashboard.putNumber("Right Drive Downtiplier", leftDowntiplier);
        SmartDashboard.putNumber("Left Drive Downtiplier", rightDowntiplier);
    }
    
    private void setSideVoltages(double left, double right) {
        final double leftOutput = (left * currentSpeedMultipler + (Math.signum(MathUtil.applyDeadband(left, 12e-2))) * 12 * 0.0175) * MAX_SPEED * (1 - leftDowntiplier);
        final double rightOutput = (right * currentSpeedMultipler + (Math.signum(MathUtil.applyDeadband(right, 12e-2))) * 12 * 0.0105) * MAX_SPEED * (1 - rightDowntiplier);
        lastDriveVoltages = new DifferentialDriveWheelSpeeds(leftOutput, rightOutput);
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

        setSidePercentages(leftMotorOutput, rightMotorOutput);
    }

    public void drivePercentage(DifferentialDriveWheelSpeeds wheelSpeeds){
        setSidePercentages(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    public void driveMetersPerSecond(DifferentialDriveWheelSpeeds wheelSpeeds) {
        double rightVoltage = rightFeedforward.calculate(wheelSpeeds.rightMetersPerSecond);
        rightVoltage += rightPID.calculate(rEncoder.getRate(), wheelSpeeds.rightMetersPerSecond);

        double leftVoltage = leftFeedforward.calculate(wheelSpeeds.leftMetersPerSecond);
        leftVoltage += leftPID.calculate(lEncoder.getRate(), wheelSpeeds.leftMetersPerSecond);

        setSideVoltages(rightVoltage, leftVoltage);
    }

    public void drive(ChassisSpeeds speeds){
        // Convert to wheel speeds
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        driveMetersPerSecond(wheelSpeeds);
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
        m_field.setRobotPose(getPose());
        SmartDashboard.putData("Field", m_field);
        if (INCLUDE_LIMELIGHT) poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d(LIMELIGHT_NAME), Timer.getFPGATimestamp());
        MAX_SPEED = SmartDashboard.getNumber("Max Speed", 1);
        SmartDashboard.putNumber("left vel", lEncoder.getRate());
        SmartDashboard.putNumber("right vel", rEncoder.getRate());
        SmartDashboard.putNumber("left pos", lEncoder.getDistance());
        SmartDashboard.putNumber("right pos", rEncoder.getDistance());


        rightDowntiplier = SmartDashboard.getNumber("Right Drive Downtiplier", leftDowntiplier);
        leftDowntiplier = SmartDashboard.getNumber("Left Drive Downtiplier", rightDowntiplier);

        //rightFriction = SmartDashboard.getNumber("Left Drive Static Friction", 0);  
        //leftFriction = SmartDashboard.getNumber("Right Drive Static Friction", 0);
        if (switchableDriverCam != null) {
            switchableDriverCam.setStreamToIndex(kinematics.toChassisSpeeds(lastDriveVoltages).vxMetersPerSecond >= 0 ? 0 : 1); // magnitude won't be right from this, but sign will be, so I don't care
        }
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
     * @param drivetrain drivetrain object
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
            public void initialize() {
                System.out.println("drivetrain init");
            }

            @Override
            public void execute() {
                setSidePercentages(leftSpeed, rightSpeed);
                System.out.println("Driving");
            }

            @Override
            public void end(boolean interrupted) {
                setSidePercentages(0, 0);
                System.out.println("Stop driving");
            }
        };
    }
    
}
