package frc.robot.commands.shooter;

import static frc.robot.Constants.LIMELIGHT_NAME;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.util.LimelightHelpers;

public class ShooterPitchControlCommand extends Command{

    // all constants in metric
    public static final double NOTE_SPEED = 10; // initial velocity, m/s
    public static final double SPEAKER_HEIGHT = 2.05; // m
    public static final double GRAVITY = 9.80441715516; // m/s/s
    public static final double SHOOTER_HEIGHT = 0.1524; // m
    public static final double TARGET_TAG_ID = 7; // April Tag ID

    public static double theta; // radians
    public static double speakerDistance; // m
    public static double lastSpeakerDistance; // m

    public final Arm arm;

    public ShooterPitchControlCommand(Arm arm){
        this.arm = arm;
        this.addRequirements(this.arm);
    }

    @Override
    public void execute(){
        if(!LimelightHelpers.getTV(LIMELIGHT_NAME)){
            // If April Tag is not visible, set speaker distance to last recorded speaker distance
            speakerDistance = lastSpeakerDistance; 
        }else{
            // Otherwise, verify April Tag ID is correct and then set speaker distance to x transform of April Tag relative to camera (distance from April Tag to camera)
            if(LimelightHelpers.getFiducialID(LIMELIGHT_NAME) == TARGET_TAG_ID) speakerDistance = LimelightHelpers.getTargetPose3d_CameraSpace(LIMELIGHT_NAME).getX();
            lastSpeakerDistance = speakerDistance;
        }
        // Calculate angle of shooter given initial note speed, gravity, speaker distance, and speaker height/shooter height
        double root =  Math.sqrt(Math.pow(NOTE_SPEED, 4) - GRAVITY * (GRAVITY * Math.pow(speakerDistance, 2) + 2 * (SPEAKER_HEIGHT - SHOOTER_HEIGHT) * Math.pow(NOTE_SPEED, 2)));
        theta = Math.atan((Math.pow(NOTE_SPEED, 2) - root) / (GRAVITY * speakerDistance));
        // Set the target angle of the arm
        arm.setTargetAngle(new Rotation2d(theta));
    }
}
