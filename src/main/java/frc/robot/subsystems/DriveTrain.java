package frc.robot.subsystems;

import static frc.robot.Constants.BACK_LEFT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.BACK_RIGHT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.FRONT_LEFT_DRIVE_MOTOR_ID;
import static frc.robot.Constants.FRONT_RIGHT_DRIVE_MOTOR_ID;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorUtil;
import frc.robot.util.Utilities;

//Implemented as Ramsete (Differential)
public class DriveTrain extends SubsystemBase{
    //Track width in meters
    public static final double TRACK_WIDTH = 0;
    //Percentage
    public static final double MAX_SPEED = 1;

    
    private CANSparkMax leftLeader;
    private CANSparkMax leftFollower;

    private CANSparkMax rightLeader;
    private CANSparkMax rightFollower;

    private RelativeEncoder rEncoder, lEncoder;

    private DifferentialDriveOdometry odo;

    private ChassisSpeeds lastSpeeds;

    public DriveTrain(){
        leftLeader = new CANSparkMax(FRONT_LEFT_DRIVE_MOTOR_ID, MotorType.kBrushless);
        leftFollower = new CANSparkMax(BACK_LEFT_DRIVE_MOTOR_ID, MotorType.kBrushless);
        rightLeader = new CANSparkMax(FRONT_RIGHT_DRIVE_MOTOR_ID, MotorType.kBrushless);
        rightFollower = new CANSparkMax(BACK_RIGHT_DRIVE_MOTOR_ID, MotorType.kBrushless);

        leftLeader.setInverted(true);
        leftFollower.follow(leftLeader, false);

        rightFollower.follow(rightLeader);

        rEncoder = rightLeader.getEncoder();
        lEncoder = leftLeader.getEncoder();

        rEncoder.setPosition(0);
        lEncoder.setPosition(0);

        odo = new DifferentialDriveOdometry(new Rotation2d(), lEncoder.getPosition(), rEncoder.getPosition());

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
    }

    public void arcadeDrive(double speed, double rotation){
        // Clamp inputs
        speed = MotorUtil.clampPercent(speed) * MAX_SPEED;
        rotation = MotorUtil.clampPercent(rotation) * MAX_SPEED;

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

        leftLeader.set(leftMotorOutput);
        rightLeader.set(rightMotorOutput);   
    }

    public void drive(DifferentialDriveWheelSpeeds wheelSpeeds){
        //Should maybe clamp values
        leftLeader.set(wheelSpeeds.leftMetersPerSecond);
        rightLeader.set(wheelSpeeds.rightMetersPerSecond);
    }

    public void drive(ChassisSpeeds speeds){
        DifferentialDriveKinematics kinematics =
        new DifferentialDriveKinematics(TRACK_WIDTH);

        lastSpeeds = speeds;

        // Convert to wheel speeds
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        drive(wheelSpeeds);
    }

    private Pose2d getPose(){
        return odo.getPoseMeters();
    }

    private void resetPose(Pose2d pose){
        //Probably should implment without nulls
        odo.resetPosition(new Rotation2d(), new DifferentialDriveWheelPositions(0,0), pose);
    }

    private ChassisSpeeds getCurrentSpeeds(){
        return lastSpeeds;
    }
}
