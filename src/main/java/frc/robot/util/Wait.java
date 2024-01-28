package frc.robot.util;

import edu.wpi.first.wpilibj.RobotController;

public class Wait {
    private final double waitTimeMS;
    private double timeMS;
    private double startTime = -1;

    public Wait(Double waitTimeMS){
        this.waitTimeMS = waitTimeMS;
        this.timeMS = waitTimeMS;
    }

    /**
     * Update the timer by subtracting the values from the initial time
     * @return if the timer is over
     */
    public boolean update(){
        if(startTime == -1) startTime = RobotController.getFPGATime() * 1000;
        else timeMS -= RobotController.getFPGATime() * 1000 - startTime;

        if(timeMS <= 0) return true;
        return false;
    }

    /**
     * Reset the timer to its initial value
     */
    public void reset(){
        timeMS = waitTimeMS;
    }
}
