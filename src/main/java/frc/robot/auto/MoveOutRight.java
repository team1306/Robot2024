package frc.robot.auto;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class MoveOutRight extends MoveOutMid {
    public MoveOutRight(DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) {
        super(0, 0.3, .8, driveTrain, shooter, arm, intake, 2.7);
    }
    
    @Override
    public String getName() {
        return "Move Out Right";
    }
}
