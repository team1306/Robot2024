package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberDriveCommand extends Command {
    public enum State {
        DOWN_OFF,
        ROTATING,
        UP_OFF,
        CLIMBING
    }

    private Climber climber;
    private State state = State.DOWN_OFF;
    private Timer timer = new Timer();

    public void ClimberDriverCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize(){
        climber.setTargetRPM(0);
    }

    @Override
    public void execute() {
        switch (state) {
            case DOWN_OFF: //if down set rpm to 0
                climber.setTargetRPM(0);
            case ROTATING: //if rotating set RPM to half of max
                climber.setTargetRPM(climber.MAX_RPM/2);
                if(timer.get() > 2) { //if timer is at two seconds switch to UP_OFF
                    state = state.UP_OFF;
                }
            case UP_OFF: //if up do nothing
            case CLIMBING: //if climbing set RPM to max
                climber.setTargetRPM(climber.MAX_RPM);
        }
    }

    public void buttonPress() {
        switch (state) {
            case DOWN_OFF: //if down reset timer and switch to rotating
                timer.restart();
                state = state.ROTATING;
            case UP_OFF: state = state.CLIMBING; //switch to climbing
            case CLIMBING: state = state.UP_OFF; //switch to up_off
            
        };
    }

    @Override
    public void end(boolean interrupted) {
        initialize(); // set power back to zero
    }
}
