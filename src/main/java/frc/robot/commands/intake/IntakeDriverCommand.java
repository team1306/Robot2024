package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeDriverCommand extends Command {
    public enum State {
        UNPOWERED_NO_ELEMENT,
        POWERED_NO_ELEMENT,
        UNPOWERED_WITH_ELEMENT,
        INDEXING
    }

    private Intake intake;
    private State state = State.UNPOWERED_NO_ELEMENT;
    private Timer timer = new Timer();

    public IntakeDriverCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setTargetRPM(0);
    }

    @Override
    public void execute() {
        switch (state) {
            case POWERED_NO_ELEMENT:
                if (!intake.notePresent()) {
                    intake.setTargetRPM(.6 * Intake.MAX_RPM);
                    break;
                }
                state = State.UNPOWERED_WITH_ELEMENT;
            case UNPOWERED_NO_ELEMENT:
            case UNPOWERED_WITH_ELEMENT:
                intake.setTargetRPM(0);
                break;
            case INDEXING:
                if (timer.get() > 2) {
                    buttonPress();
                } else {
                    intake.setTargetRPM(.6 * Intake.MAX_RPM);
                }
                break;
        }
    }

    public void buttonPress() {
        state = switch (state) {
            case UNPOWERED_NO_ELEMENT -> State.POWERED_NO_ELEMENT;
            case UNPOWERED_WITH_ELEMENT -> {
                timer.reset();
                yield State.INDEXING;
            }
            // THIS COULD BE QUITE BUGGY, MAKE SURE TO TEST
            case INDEXING -> {
                if (intake.notePresent()) {
                    yield State.UNPOWERED_WITH_ELEMENT;
                } else {
                    yield State.UNPOWERED_NO_ELEMENT;
                }
            }
            case POWERED_NO_ELEMENT -> State.UNPOWERED_NO_ELEMENT;
        };
    }

    @Override
    public void end(boolean interrupted) {
        initialize(); // set power back to zero
    }
}
