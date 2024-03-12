package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.Constants.*;

public class Utilities {
    /**
     * Returns whether the Driverstation is on the red alliance
     * @return true if on the red alliance, false if on the blue alliance or the alliance is not present
     */
    public static boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? (alliance.get() == DriverStation.Alliance.Red) : true;
    }
    /**
     * Returns where the speaker is on the field
     * @return a {@link Translation2d} that represents where the alliance speaker is on the field
     */

    public static Translation2d getSpeaker() {
        return Utilities.isRedAlliance() ? RED_SPEAKER : BLUE_SPEAKER;
    }

    /**
     * Gets the speaker distance depending on the side the driverstation is on
     * @param robotPos the pos of the robot
     * @return the distance in meters
     */

    public static double getSpeakerDistance(Pose2d robotPos){
        return robotPos.getTranslation().getDistance(Utilities.getSpeaker());
    }

    /**
     * Gets the robot pos based on the limelight
     * @return a pos2d where the robot is
     */
    public static Pose2d getRobotPos(){
        return INCLUDE_LIMELIGHT ? LimelightHelpers.getBotPose2d(LIMELIGHT_NAME) : new Pose2d();
    }
}
