package frc.robot.commands.outreach;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    private enum State{
        POWERED,
        UNPOWERED,
        WITH_ELEMENT,
        REVERSING
    }
    public static final double INTAKE_SPEED = 0.6;
    private final Intake intake;

    private State state = State.UNPOWERED;
    
    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public void execute(){
        state = intake.notePresent() ? State.WITH_ELEMENT : state;

        switch(state){
            case POWERED:
                intake.setTargetSpeed(INTAKE_SPEED);
                break;
            case WITH_ELEMENT:
                if(!intake.notePresent()) state = State.UNPOWERED;
            case UNPOWERED:
                intake.setTargetSpeed(0);
                break;
            case REVERSING:
                intake.setTargetSpeed(-INTAKE_SPEED);
                break;
        }
    }

    public void buttonPress(){
        state = switch(state){
            case POWERED -> State.UNPOWERED;
            case UNPOWERED -> State.POWERED;

            case WITH_ELEMENT -> State.WITH_ELEMENT;
            case REVERSING -> State.UNPOWERED;
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
