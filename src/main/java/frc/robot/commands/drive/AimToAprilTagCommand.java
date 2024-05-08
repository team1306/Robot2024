package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class AimToAprilTagCommand extends Command {
    public enum State {
        SEARCHING,
        TRACKING,
        LOCKED;
    }

    private final DriveTrain driveTrain;
    private final Shooter shooter;
    private final Intake intake;
    private PIDController rotationController;

    private State state = State.SEARCHING;

    public static double kP = 0.0185, kI = 0, kD = 2.2e-3;
    public static final double TOLERANCE_DEGREES = 1;

    public AimToAprilTagCommand(DriveTrain driveTrain, Shooter shooter, Intake intake){
        this.driveTrain = driveTrain;
        this.shooter = shooter;
        this.intake = intake;
        this.addRequirements(driveTrain);
    }

    /**
     * called every time command is scheduled to ensure that old values do not mess anything up
     */
    @Override
    public void initialize(){
        this.rotationController = new PIDController(kP, kI, kD);
        rotationController.setTolerance(TOLERANCE_DEGREES);
    }

    @Override
    public void execute() {
        rotationController.setPID(kP, kI, kD);

        boolean targetVisible = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false);
        double xOffset = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        boolean targetRight = false;

        double outputPower;

        switch (state) {
            case SEARCHING:
                if (targetRight) outputPower = 0.5; //if last seen to the right, turn right
                else outputPower = -0.5; //if last seen to the left, turn left
                if (targetVisible) state = State.TRACKING; //if it sees the target, switch to tracking mode
                break;

            case TRACKING:
                outputPower = MathUtil.clamp(rotationController.calculate(xOffset, 0), -0.5, 0.5);
                driveTrain.arcadeDrive(0, outputPower); //physically turn

                //tracking where target was last seen
                targetRight = xOffset > 0;

                if (-2 < xOffset && xOffset < 2) state = State.LOCKED; //if crosshair is on target, switch to locked mode
                else if (!targetVisible) state = State.SEARCHING; //if target leaves the screen, switch to searching
                break;

            case LOCKED:
                outputPower = MathUtil.clamp(rotationController.calculate(xOffset, 0), -0.5, 0.5);
                driveTrain.arcadeDrive(0, outputPower); //physically turn
                shooter.setTargetSpeed(.70); //spin up shooter; make configurable
                if (shooter.getTopRPM() > 4000) //if shooter up to speed
                    new IntakeIndexCommand(intake).schedule(); //FIRE!
                if (!(-2 < xOffset && xOffset < 2)) state = State.LOCKED; //if crosshair is not on target, switch to tracking mode
                break;
        }
    }

    @Override
    public boolean isFinished() {            
        return false; //idk when to make it finish
    }
}
