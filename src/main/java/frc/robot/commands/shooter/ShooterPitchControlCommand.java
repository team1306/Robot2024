package frc.robot.commands.shooter;

import static frc.robot.Constants.INCLUDE_LIMELIGHT;
import static frc.robot.Constants.LIMELIGHT_NAME;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Arm;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utilities;

public class ShooterPitchControlCommand extends Command{

    public static final double SHOOTER_X_OFFSET = 0; // from camera, O_x
    public static final double SHOOTER_DEGREE_OFFSET = 0; // shooter angle offset, radians, c
    private final ShooterDriveCommand shooterDriveCommand;

    public final Arm arm;

    public double theta; // arm angle, radians
    public double speakerDistance; // m, d_s
    
    private double a = 1, b = 1, c = 1;

    public ShooterPitchControlCommand(Arm arm, ShooterDriveCommand shooterDriveCommand){
        this.shooterDriveCommand = shooterDriveCommand;
        this.arm = arm;
        this.addRequirements(this.arm);
    }

    @Override
    public void initialize(){
        Pose2d botPose = INCLUDE_LIMELIGHT ? LimelightHelpers.getBotPose2d(LIMELIGHT_NAME) : new Pose2d();
        speakerDistance = botPose.getTranslation().getDistance(Utilities.getSpeaker());
        speakerDistance += SHOOTER_X_OFFSET;        
        
        //Theta must be in terms of degrees
        theta = a * Math.pow(speakerDistance, 2) + b * speakerDistance + c;
        // Set the target angle of the arm
        arm.setTargetAngle(Rotation2d.fromDegrees(theta + SHOOTER_DEGREE_OFFSET));
        CommandScheduler.getInstance().schedule(shooterDriveCommand);    
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
