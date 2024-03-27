package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
@Deprecated
public class ClimberDriverCommand extends Command {

    private static double CLIMBER_SPEED = .7; //speed of the climber as a decimal
    private static double MAX_CLIMBER_ANGLE = 5; //angle that the climber stops in degrees
    
    public enum State {
        DOWN_OFF,
        ROTATING,
        UP_OFF,
        CLIMBING
    }

    private Climber climber;
    
    private State state = State.DOWN_OFF;
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

    @Override
    public void execute() {
        MAX_CLIMBER_ANGLE = SmartDashboard.getNumber("Max climber angle", MAX_CLIMBER_ANGLE);
        CLIMBER_SPEED = SmartDashboard.getNumber("Speed for the climber", CLIMBER_SPEED);
        final boolean leftUpVal = leftUp.getAsBoolean(), leftDownVal = leftDown.getAsBoolean(), rightUpVal = rightUp.getAsBoolean(), rightDownVal = rightDown.getAsBoolean(),
                buttonPressed = leftUpVal || leftDownVal || rightUpVal || rightDownVal;

        if (buttonPressed) { //when any override button is pressed
            state = State.DOWN_OFF;
            if (leftUpVal) {
               climber.setLeftSpeed(CLIMBER_SPEED);
            } else if (leftDownVal) {
                climber.setLeftSpeed(0);
            }
            if (rightUpVal) {
                climber.setRightSpeed(CLIMBER_SPEED);
            } else if (rightDownVal) {
                climber.setRightSpeed(0);
            }
            return;
        }
        switch (state) {
            case DOWN_OFF: //if down set rpm to 0
                climber.setTargetSpeed(0);
                climber.zeroEncoders();
                break;
            case ROTATING: //if rotating set RPM to half of max
                climber.setTargetSpeed(CLIMBER_SPEED);
                if (climber.getRightAngle().getRotations() > MAX_CLIMBER_ANGLE || climber.getLeftAngle().getRotations() > MAX_CLIMBER_ANGLE) {
                    state = State.UP_OFF;
                }
                break;
            case UP_OFF: //if up do nothing
                climber.setTargetSpeed(0);
                break;
            case CLIMBING: //if climbing set RPM to faster than normal
                climber.setTargetSpeed(CLIMBER_SPEED*2);
                break;
        }
    }

    public void buttonPress() {
        switch (state) {
            case DOWN_OFF: //if down reset timer and switch to rotating
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

    public void resetState() {
        state = State.DOWN_OFF;
    }
}
