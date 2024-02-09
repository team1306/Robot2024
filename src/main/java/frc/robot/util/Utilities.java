package frc.robot.util;

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
}
