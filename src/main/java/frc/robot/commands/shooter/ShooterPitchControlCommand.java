package frc.robot.commands.shooter;

import static frc.robot.Constants.LIMELIGHT_NAME;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drive.ShooterDriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utilities;

public class ShooterPitchControlCommand extends Command{

    // all constants in metric
    public static final double SPEAKER_HEIGHT = 2.05; // m
    public static final double GRAVITY = 9.80441715516; // m/s/s
    public static final double SHOOTER_X_OFFSET = 0; // from camera, O_x
    public static final double SHOOTER_Y_OFFSET = 0; // from camera, O_y
    public static final double SHOOTER_RADIAN_OFFSET = 0; // shooter angle offset, radians, c
    public static final double SHOOTER_RADIUS = 0; // meters, r

    public final Arm arm;
    private final ShooterDriveCommand shooterDriveCommand;

    public double phi; // arm angle, radians
    public double speakerDistance; // m, d_s
    public double lastSpeakerDistance; // m

    public ShooterPitchControlCommand(Arm arm, ShooterDriveCommand shooterDriveCommand){
        this.shooterDriveCommand = shooterDriveCommand;
        this.arm = arm;
        this.addRequirements(this.arm);
    }

    @Override
    public void initialize(){
        Pose2d botPose = LimelightHelpers.getBotPose2d(LIMELIGHT_NAME);
        speakerDistance = botPose.getTranslation().getDistance(Utilities.getSpeaker());

        // Calculate angle of shooter given initial note speed, gravity, speaker distance, and speaker height/shooter height
        double phi = Math.PI - (SHOOTER_RADIAN_OFFSET + Math.asin((SHOOTER_RADIUS*Math.sin(SHOOTER_RADIAN_OFFSET))/Math.sqrt(Math.pow((SHOOTER_X_OFFSET+speakerDistance), 2)+Math.pow((SPEAKER_HEIGHT-SHOOTER_Y_OFFSET), 2)))) + Math.atan(SPEAKER_HEIGHT/(SHOOTER_X_OFFSET+speakerDistance)); 
        
        // Set the target angle of the arm
        arm.setTargetAngle(new Rotation2d(phi));
        CommandScheduler.getInstance().schedule(shooterDriveCommand);    
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
