package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberDriverCommand extends Command {

    private static double ROTATING_TIME = 2;
    private static double CLIMBER_SPEED = 0.1; //speed of the climber as a decimal
    
    public enum State {
        DOWN_OFF,
        ROTATING,
        UP_OFF,
        CLIMBING
    }

    private Climber climber;
    
    private State state = State.DOWN_OFF;
    private Timer timer = new Timer();
    private final BooleanSupplier leftUp, rightUp, leftDown, rightDown;

    public ClimberDriverCommand(Climber climber, BooleanSupplier leftUp, BooleanSupplier rightUp, BooleanSupplier leftDown, BooleanSupplier rightDown) {
        this.climber = climber;

        this.leftUp = leftUp;
        this.leftDown = leftDown;
        this.rightUp = rightUp;
        this.rightDown = rightDown;

        addRequirements(climber);
    }

    @Override
    public void initialize(){
        climber.setTargetSpeed(0);
    }

    public double leftEncoderValue () {
        return 0;
    }
    public double rightEncoderValue () {
        return 0;
    }

    @Override
    public void execute() {
        final boolean leftUpVal = leftUp.getAsBoolean(), leftDownVal = leftDown.getAsBoolean(), rightUpVal = rightUp.getAsBoolean(), rightDownVal = rightDown.getAsBoolean(),
                buttonPressed = leftUpVal || leftDownVal || rightUpVal || rightDownVal;
        if (buttonPressed) { //when any override button is pressed
            state = State.DOWN_OFF;
            if (leftUpVal) {
                climber.setLeftSpeed(CLIMBER_SPEED);
            }
            if (leftDownVal) {
                climber.setLeftSpeed(0);
            }
            if (rightUpVal) {
                climber.setRightSpeed(CLIMBER_SPEED);
            }
            if (rightDownVal) {
                climber.setRightSpeed(0);
            }
        }
        switch (state) {
            case DOWN_OFF: //if down set rpm to 0
                climber.setTargetSpeed(0);
                climber.zeroEncoders();
                break;
            case ROTATING: //if rotating set RPM to half of max
                climber.setTargetSpeed(CLIMBER_SPEED);
                if (climber.getRightAngle() > 90 && climber.getRightAngle() > 90) {
                    state = State.UP_OFF;
                }
                break;
            case UP_OFF: //if up do nothing
                break;
            case CLIMBING: //if climbing set RPM to faster than normal
                climber.setTargetSpeed(CLIMBER_SPEED*2);
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
