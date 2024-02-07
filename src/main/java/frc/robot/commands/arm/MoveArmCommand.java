public class MoveArmCommand extends Command {
    private final Arm arm;
    private final XboxController xboxController;

    private int minAngle = 0;
    private int maxAngle = 90;

    private double rotationSpeed;

    public MoveArmCommand(DriveTrain driveTrain, XboxController xboxController){
        this.Arm = arm;
        this.xboxController = xboxController;
        this.addRequirements(arm);
    }

    @Override
    public void execute(){
        rotationSpeed = xboxController.getRightTriggerAxis();
        rotationSpeed *= (rotationSpeed < minAngle || rotationSpeed > maxAngle ? 0.1 : 1);
        arm.setTargetAngle(arm.getCurrentAngle() + rotationSpeed);
    }
}