package frc.robot.commands.drive;

import static frc.robot.Constants.LIMELIGHT_NAME;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.commands.intake.IntakeDriverCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.DashboardGetter;
import frc.robot.util.LimelightHelpers;

public class AimBotCommand extends Command{
    private enum State{
        Searching,
        Tracking,
        Locked,
        Shooting,
        Idle,
        Driving
    }

    private final DriveTrain driveTrain;
    private final Shooter shooter;
    private final Intake intake;
    private final Arm arm;
    private final IntakeDriverCommand intakeDriverCommand;
    
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier rotationSupplier;

    private State state = State.Idle;

    public static double kP = 0.1, kI = 0, kD = 0;
    private PIDController rotationController = new PIDController(kP, kI, kD);

    public double toleranceDegrees = 1;

    private Timer timer = new Timer();

    private boolean lastTargetRight = false;
    private boolean timerStarted = false;

    public AimBotCommand(DriveTrain driveTrain, Shooter shooter, Intake intake, Arm arm, DoubleSupplier leftYSupplier, DoubleSupplier rotationSupplier){
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.intake = intake;
        this.arm = arm;
        this.intakeDriverCommand = new IntakeDriverCommand(intake, shooter, () -> false, () -> arm.getCurrentAngle().getDegrees(), IntakeDriverCommand.State.UNPOWERED_NO_ELEMENT);
        this.forwardSupplier = leftYSupplier;
        this.rotationSupplier = rotationSupplier;
        
        DashboardGetter.addGetDoubleData("Locking Range", toleranceDegrees, (value)-> toleranceDegrees = value);

        addRequirements(driveTrain, shooter);
    }

    private void armDown() {
        new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.INTAKE).schedule();
    }
    
    private void armUp() {
        new MoveArmToSetpointCommand(arm, () -> 60).schedule();
    }

    public void initialize(){
        System.out.println("INit");
        intakeDriverCommand.schedule();
    }

    public void execute(){
        double rotation = 0;
        double forward = 0;

        boolean targetVisible = LimelightHelpers.getTV(LIMELIGHT_NAME);
        double xOffset = LimelightHelpers.getTX(LIMELIGHT_NAME);

        if(Math.abs(forwardSupplier.getAsDouble()) < 0.05 || Math.abs(rotationSupplier.getAsDouble()) < 0.05 )
            state = State.Driving;
        
        switch(state){
            case Driving:
                forward = Math.copySign(Math.pow(forwardSupplier.getAsDouble(), 2), forwardSupplier.getAsDouble());
                rotation = Math.copySign(Math.pow(rotationSupplier.getAsDouble(), 2), rotationSupplier.getAsDouble());
                break;

            case Idle:
                armDown();
                shooter.setTargetSpeed(0);
                intakeDriverCommand.setState(intake.notePresent() ? IntakeDriverCommand.State.UNPOWERED_WITH_ELEMENT : IntakeDriverCommand.State.UNPOWERED_NO_ELEMENT);
                break;
            
            case Locked:
                if(shooter.getTargetSpeed() == 0 && intake.notePresent() && targetVisible) {
                    shooter.setTargetSpeed(0.7);
                    armUp();
                    timerStarted = false;
                    state = State.Shooting;
                }
                else if (!targetVisible) state = State.Searching;
                else if (Math.abs(xOffset) > toleranceDegrees) state = State.Tracking;
                break;

            case Searching:
                if (targetVisible){
                    state = State.Tracking;
                    break;
                } //if it sees the target, switch to tracking mode
                if (lastTargetRight) rotation = 0.3; //if last seen to the right, turn right
                else rotation = -0.3; //if last seen to the left, turn left
                break;

            case Shooting:
                if(shooter.getBottomRPM() > 1500){
                    intakeDriverCommand.setState(IntakeDriverCommand.State.INDEXING);
                    if(!timerStarted) {
                        timer.restart();
                        timerStarted = true;
                    }
                    if(timer.hasElapsed(1))
                        state = State.Idle;
                }
                break;
            
            case Tracking:
                //if crosshair is on target, switch to locked mode    
                if (Math.abs(xOffset) < toleranceDegrees) { 
                    state = State.Locked; 
                    break;
                } 
                //if target leaves the screen, switch to searching
                else if (!targetVisible){ 
                    state = State.Searching; 
                    break;
                }
                rotation = MathUtil.clamp(rotationController.calculate(xOffset), -0.5, 0.5);
                break;
        }

        if (targetVisible) lastTargetRight = 0 < xOffset;

        SmartDashboard.putNumber("Supervise Rotation", rotation);
        System.out.println(rotation);
        driveTrain.arcadeDrive(forward, rotation);
    }

    public void switchState(){
        state = switch(state){
            default -> State.Idle;
            case Idle -> State.Searching;
        };
        System.out.println(state);
    }

    public void reload(){
        intakeDriverCommand.buttonPress();
        System.out.println("Reloading");
    }

    public boolean isFinished(){
        return false;
    }
}
