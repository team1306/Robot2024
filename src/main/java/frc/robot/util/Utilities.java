package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class Utilities {
    /**
     * Returns whether the Driverstation is on the red alliance
     * @return true if on the red alliance, false if on the blue alliance or the alliance is not present
     */
    public static boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? (alliance.get() == DriverStation.Alliance.Red) : true;
    }
}
