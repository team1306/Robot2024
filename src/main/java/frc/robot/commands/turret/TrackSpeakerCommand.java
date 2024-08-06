package frc.robot.commands.turret;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.util.Utilities;

public class TrackSpeakerCommand extends Command{
    private final Turret turret;
    private final SwerveDrivePoseEstimator estimator;
    public TrackSpeakerCommand(Turret turret, SwerveDrivePoseEstimator estimator){
        this.turret = turret;
        this.estimator = estimator;
    }

    public void execute(){
        Pose2d pose = estimator.getEstimatedPosition();
        Translation2d speaker = Utilities.getSpeaker();
        Rotation2d relativeAngle = Rotation2d.fromRadians(Math.PI / 2 - Math.atan(speaker.getY() / speaker.getX())).plus(pose.getRotation());
        MoveTurretToRelativeAngle moveTurretToRelativeAngle = new MoveTurretToRelativeAngle(turret, relativeAngle);
        moveTurretToRelativeAngle.schedule();
    }

    public boolean isFinished(){
        return false;
    }
}
