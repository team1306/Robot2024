package frc.robot.commands.shooter;

import static frc.robot.Constants.BLUE_SPEAKER;
import static frc.robot.Constants.LIMELIGHT_NAME;
import static frc.robot.Constants.RED_SPEAKER;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utilities;

public class ShooterPitchControlCommand extends Command{

    // all constants in metric
    public static final double NOTE_SPEED = 10; // initial velocity, m/s
    public static final double SPEAKER_HEIGHT = 2.05; // m
    public static final double GRAVITY = 9.80441715516; // m/s/s
    public static final double SHOOTER_HEIGHT = 0.1524; // m

    public static double theta; // radians
    public static double speakerDistance; // m
    public static double lastSpeakerDistance; // m

    public final Arm arm;

    public ShooterPitchControlCommand(Arm arm) {
        this.arm = arm;
        this.addRequirements(this.arm);
    }

    @Override
    public void execute(){
        Pose2d botPose = LimelightHelpers.getBotPose2d(LIMELIGHT_NAME);
        speakerDistance = botPose.getTranslation().getDistance(Utilities.isRedAlliance() ? RED_SPEAKER : BLUE_SPEAKER);

        // Calculate angle of shooter given initial note speed, gravity, speaker distance, and speaker height/shooter height
        double root =  Math.sqrt(Math.pow(NOTE_SPEED, 4) - GRAVITY * (GRAVITY * Math.pow(speakerDistance, 2) + 2 * (SPEAKER_HEIGHT - SHOOTER_HEIGHT) * Math.pow(NOTE_SPEED, 2)));
        theta = Math.atan((Math.pow(NOTE_SPEED, 2) - root) / (GRAVITY * speakerDistance));
        // Set the target angle of the arm
        arm.setTargetAngle(new Rotation2d(theta));
    }
}
