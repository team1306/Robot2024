package frc.robot.commands.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeDriverCommand extends Command {
    public enum State {
        UNPOWERED_NO_ELEMENT,
        POWERED_NO_ELEMENT,
        REVERSING,
        UNPOWERED_WITH_ELEMENT,
        INDEXING
    }

    private Intake intake;
    private State state = State.UNPOWERED_NO_ELEMENT;
    private Timer timer = new Timer();
    private BooleanSupplier reverseOverride;
    private boolean wasReversed = false;
    public static final double INTAKE_SPEED = 0.6;
    private int noteNotPresentConfidence = 0;
    public IntakeDriverCommand(Intake intake, BooleanSupplier reverseOverride) {
        this.intake = intake;
        this.reverseOverride = reverseOverride;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setTargetSpeed(0);
    }

    public void setState(State state) {
        this.state = state;
    }

    @Override
    public void execute() {
        if (reverseOverride.getAsBoolean()) {
            intake.setTargetSpeed(-1);
            wasReversed = true;
            return;
        } else if (wasReversed) {
            state = State.UNPOWERED_NO_ELEMENT;
            wasReversed = false;
        }
        switch (state) {
            case POWERED_NO_ELEMENT:
                if (!intake.notePresent()) {
                    intake.setTargetSpeed(INTAKE_SPEED);
                    break;
                }
                state = State.REVERSING;
                intake.setTargetSpeed(-1.0 / 8.0);
                timer.restart();
            case REVERSING:
                if (timer.get() > 0.36) {
                    intake.setTargetSpeed(0);
                    state = State.UNPOWERED_WITH_ELEMENT;
                }
                noteNotPresentConfidence = 0;
                break;
            case UNPOWERED_NO_ELEMENT:
                intake.setTargetSpeed(0);
            case UNPOWERED_WITH_ELEMENT:
                intake.setTargetSpeed(0);
                if (!intake.notePresent()) {
                    ++noteNotPresentConfidence;
                }
                if (noteNotPresentConfidence >= 5) {
                    state = State.UNPOWERED_NO_ELEMENT;
                }
                break;
            case INDEXING:
                if (timer.hasElapsed(.5)) {
                    buttonPress();
                } else {
                    intake.setTargetSpeed(INTAKE_SPEED);
                }
                break;
        }
    }

    public void buttonPress() {
        state = switch (state) {
            case UNPOWERED_NO_ELEMENT -> State.POWERED_NO_ELEMENT;
            case POWERED_NO_ELEMENT -> State.UNPOWERED_NO_ELEMENT;
            case UNPOWERED_WITH_ELEMENT -> {
                timer.restart();
                yield State.INDEXING;
            }
            // THIS COULD BE QUITE BUGGY, MAKE SURE TO TEST
            case INDEXING -> intake.notePresent() ? State.UNPOWERED_WITH_ELEMENT : State.UNPOWERED_NO_ELEMENT;
            case REVERSING -> State.REVERSING; // loop
        };
    }

    public void clearNote() {
        state = State.UNPOWERED_NO_ELEMENT;
    }

    @Override
    public void end(boolean interrupted) {
        initialize(); // set power back to zero
    }
}
