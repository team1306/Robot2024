package frc.robot.commands.outreach;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    private enum State{
        POWERED,
        UNPOWERED,
        WITH_ELEMENT,
        WITH_ELEMENT_REVERSE,
        REVERSING
    }
    public static final double INTAKE_SPEED = 0.6;
    private final Intake intake;

    private State state = State.UNPOWERED;
    private Timer timer = new Timer();
    
    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public void execute(){
        if (intake.notePresent() && (state == State.UNPOWERED || state == State.POWERED)) {
            state = State.WITH_ELEMENT_REVERSE;
            timer.restart();
        }

        switch(state){
            case POWERED:
                intake.setTargetSpeed(INTAKE_SPEED);
                break;
            case WITH_ELEMENT:
                if(!intake.notePresent()) state = State.UNPOWERED;
            case UNPOWERED:
                intake.setTargetSpeed(0);
                break;
            case WITH_ELEMENT_REVERSE:
                intake.setTargetSpeed(-1/8D);
                if(timer.get() > 0.25) state = State.WITH_ELEMENT;
                break;
            case REVERSING:
                intake.setTargetSpeed(-1);
                break;
        
        }
    }

    public void buttonPress(){
        state = switch(state){
            case POWERED -> State.UNPOWERED;
            case UNPOWERED -> State.POWERED;

            case WITH_ELEMENT -> State.WITH_ELEMENT;
            case REVERSING -> State.UNPOWERED;

            case WITH_ELEMENT_REVERSE -> State.WITH_ELEMENT_REVERSE;
        };
    }

    public void reverseOverride(){
        state = switch(state){
            case REVERSING -> State.UNPOWERED;
            case UNPOWERED -> State.REVERSING;

            default -> State.REVERSING;
        };
    }

    public void end(boolean interrupted){
        intake.setTargetSpeed(0);
    }
}
