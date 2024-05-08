package frc.robot.commands.outreach;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    private enum State{
        POWERED,
        UNPOWERED,
        WITH_ELEMENT
    }
    public static final double INTAKE_SPEED = 0.6;
    private final Intake intake;
    private final BooleanSupplier reverseOverride;
    private boolean wasReversed = false;

    private State state;
    
    public IntakeCommand(Intake intake, BooleanSupplier reverseOverride) {
        this.intake = intake;
        this.reverseOverride = reverseOverride;

        addRequirements(intake);
    }

    public void execute(){
        if (reverseOverride.getAsBoolean()) {
            intake.setTargetSpeed(-INTAKE_SPEED);
            wasReversed = true;
            return;
        } else if (wasReversed) {
            state = State.UNPOWERED;
            wasReversed = false;
        }

        switch(state){
            case POWERED:
                intake.setTargetSpeed(INTAKE_SPEED);
                if(intake.notePresent()){
                    state = State.WITH_ELEMENT;
                }
                break;
            case UNPOWERED:
            case WITH_ELEMENT:
                intake.setTargetSpeed(0);
                break;
        }
    }

    public void buttonPress(){
        state = switch(state){
            case POWERED -> State.UNPOWERED;
            case UNPOWERED -> State.POWERED;

            case WITH_ELEMENT -> State.WITH_ELEMENT;
        };
    }

    public void end(boolean interrupted){
        intake.setTargetSpeed(0);
    }
}
