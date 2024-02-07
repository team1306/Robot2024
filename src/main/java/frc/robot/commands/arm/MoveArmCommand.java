package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;

public class MoveArmCommand extends Command {
    private final Arm arm;
    private final XboxController xboxController;

    private int minAngle = 0;
    private int maxAngle = 90;

    private double rotationSpeed;

    public MoveArmCommand(DriveTrain driveTrain, Arm arm, XboxController xboxController){
        this.arm = arm;
        this.xboxController = xboxController;
        this.addRequirements(arm);
    }

    @Override
    public void execute(){
        rotationSpeed = xboxController.getRightTriggerAxis();
        rotationSpeed *= (rotationSpeed < minAngle || rotationSpeed > maxAngle ? 0.1 : 1);
        arm.setTargetAngle(arm.getCurrentAngle().plus(new Rotation2d(rotationSpeed)));
    }
}