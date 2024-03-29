package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.util.Utilities;

public abstract class BaseAuto extends SequentialCommandGroup {
    public BaseAuto(DriveTrain driveTrain, Pose2d startingPose) {
        driveTrain.resetPose(startingPose);
        driveTrain.resetGyro(Utilities.isRedAlliance() ? 180 : 0);
    }
}
