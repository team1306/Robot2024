package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeDriverCommand extends Command {
    public enum State {
        UNPOWERED_NO_ELEMENT,
        POWERED_NO_ELEMENT,
        REVERSING,
        UNPOWERED_WITH_ELEMENT,
        INDEXING
    }

    private final Intake intake;
    private final Shooter shooter;
    private State state = State.UNPOWERED_NO_ELEMENT;
    private final Timer timer = new Timer();
    private final BooleanSupplier reverseOverride;
    private boolean wasReversed = false;
    public static final double INTAKE_SPEED = 0.6;
    private int noteNotPresentConfidence = 0;
    private final DoubleSupplier armAngle;
    public IntakeDriverCommand(Intake intake, Shooter shooter, BooleanSupplier reverseOverride, DoubleSupplier armAngle) {
        this.intake = intake;
        this.reverseOverride = reverseOverride;
        this.shooter = shooter;
        this.armAngle = armAngle;
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
                if (timer.get() > 0.26) {
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
                if ((armAngle.getAsDouble() > 70 && Math.abs(shooter.getTopRPM()) >= 60) || Math.abs(shooter.getTopRPM()) >= 1600) {
                    timer.restart();
                    yield State.INDEXING;
                } else {
                    yield State.UNPOWERED_WITH_ELEMENT;
                }
            }
            // THIS COULD BE QUITE BUGGY, MAKE SURE TO TEST
            case INDEXING -> reset();
            case REVERSING -> State.REVERSING; // loop
        };
    }

    public void clearNote() {
        state = State.UNPOWERED_NO_ELEMENT;
    }

    public State reset() {
        state = intake.notePresent() ? State.UNPOWERED_WITH_ELEMENT : State.UNPOWERED_NO_ELEMENT;
        return state;
    }

    @Override
    public void end(boolean interrupted) {
        initialize(); // set power back to zero
    }
}
