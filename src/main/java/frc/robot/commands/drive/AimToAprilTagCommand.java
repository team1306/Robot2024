package frc.robot.commands.drive;

import static frc.robot.Constants.LIMELIGHT_NAME;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.commands.intake.IntakeDriverCommand;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.commands.outreach.FireCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.DashboardGetter;
import frc.robot.util.LimelightHelpers;


public class AimToAprilTagCommand extends Command {
    public enum State {
        IDLE,
        RELOADING,
        SEARCHING,
        TRACKING,
        LOCKED;
    }

    private final DriveTrain driveTrain;
    private final Shooter shooter;
    private final Intake intake;
    private final Arm arm;
    private final IntakeDriverCommand intakeDriverCommand;
    private PIDController rotationController;

    private State state = State.IDLE;

    public static double kP = 0.0185, kI = 0, kD = 2.2e-3;
    public static double toleranceDegrees = 1;

    private boolean targetRight = false;


    public AimToAprilTagCommand(DriveTrain driveTrain, Shooter shooter, Intake intake, Arm arm){
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.intake = intake;
        this.arm = arm;
        this.intakeDriverCommand = new IntakeDriverCommand(intake, shooter, () -> arm.getCurrentAngle().getDegrees(), IntakeDriverCommand.State.UNPOWERED_NO_ELEMENT);
        this.addRequirements(driveTrain, shooter, arm);

        DashboardGetter.addGetDoubleData("Locking Range", toleranceDegrees, (value)-> toleranceDegrees = value);
        armDown();
    }
    
    private void armDown() {
        new MoveArmToSetpointCommand(arm, Arm.SetpointOptions.INTAKE).schedule();
    }
    
    private void armUp() {
        new MoveArmToSetpointCommand(arm, () -> 60).schedule();
    }


    /**
     * called every time command is scheduled to ensure that old values do not mess anything up
     */
    @Override
    public void initialize(){
        intakeDriverCommand.schedule();
        this.rotationController = new PIDController(kP, kI, kD);
    }

    @Override
    public void execute() {
        rotationController.setPID(kP, kI, kD);
        rotationController.setTolerance(toleranceDegrees);

        boolean targetVisible = LimelightHelpers.getTV(LIMELIGHT_NAME);
        double xOffset = LimelightHelpers.getTX(LIMELIGHT_NAME);

        double outputPower;

        switch (state) {
            case SEARCHING:
                if (targetRight) outputPower = 0.3; //if last seen to the right, turn right
                else outputPower = -0.3; //if last seen to the left, turn left
                if (targetVisible) state = State.TRACKING; //if it sees the target, switch to tracking mode
                break;

            case TRACKING:
                
                outputPower = MathUtil.clamp(rotationController.calculate(xOffset, 0), -0.5, 0.5);
                //tracking where target was last seen
                targetRight = 0 < xOffset;
    
                if (Math.abs(xOffset) < 5) state = State.LOCKED; //if crosshair is on target, switch to locked mode
                else if (!targetVisible) state = State.SEARCHING; //if target leaves the screen, switch to searching
                break;

            case LOCKED:
                outputPower = MathUtil.clamp(rotationController.calculate(xOffset, 0), -0.5, 0.5);
                if (intake.notePresent()) shooter.setTargetSpeed(0); //add wait so shooters doesn't spin down immediately after shooting.
                else shooter.setTargetSpeed(.70); //spin up shooter; make configurable
                if (shooter.getTopRPM() > 2500 && intake.notePresent() && Math.abs(xOffset) < 2) {//if shooter up to speed
                    intakeDriverCommand.setState(IntakeDriverCommand.State.INDEXING);
                    state = State.RELOADING;
                    armDown();
                }
                
                if (Math.abs(xOffset) < 10) state = State.TRACKING; //if crosshair is not on target, switch to tracking mode
                break;

            case IDLE:
                intakeDriverCommand.setState(IntakeDriverCommand.State.UNPOWERED_NO_ELEMENT); //no intaking
            case RELOADING:
                outputPower = 0;
                break;
                
                
        }
        driveTrain.arcadeDrive(0, outputPower); //physically turn
    }

    public void buttonPress() {
        switch (state) {
            case IDLE:
                state = State.RELOADING;
                intakeDriverCommand.setState(IntakeDriverCommand.State.POWERED_NO_ELEMENT); //sucking in notes
                break;
            case RELOADING:
                state = State.SEARCHING;
                armUp();
        }        
    }

    @Override
    public boolean isFinished() {            
        return false; //idk when to make it finish
    }
}
