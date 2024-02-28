package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberDriverCommand extends Command {

    private static double ROTATING_TIME = 2;

    public enum State {
        DOWN_OFF,
        ROTATING,
        UP_OFF,
        CLIMBING
    }

    private Climber climber;
    
    private State state = State.DOWN_OFF;
    private Timer timer = new Timer();

    public ClimberDriverCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize(){
        climber.setTargetSpeed(0);
    }

    @Override
    public void execute() {
        switch (state) {
            case DOWN_OFF: //if down set rpm to 0
                climber.setTargetSpeed(0);
                break;
            case ROTATING: //if rotating set RPM to half of max
                climber.setTargetSpeed(0.5);
                if(timer.hasElapsed(ROTATING_TIME)) { //if timer is at two seconds switch to UP_OFF
                    state = State.UP_OFF;
                }
                break;
            case UP_OFF: //if up do nothing
                break;
            case CLIMBING: //if climbing set RPM to max
                climber.setTargetSpeed(1);
                break;
        }
    }

    public void buttonPress() {
        switch (state) {
            case DOWN_OFF: //if down reset timer and switch to rotating
                timer.restart();
                state = State.ROTATING;
                break;
            case UP_OFF: 
                state = State.CLIMBING; //switch to climbing
                break;
            case CLIMBING: 
                state = State.UP_OFF; //switch to up_off
                break;

            case ROTATING: // default cases
            default:
                break;
        
        };
    }

    @Override
    public void end(boolean interrupted) {
        initialize(); // set power back to zero
    }
}
