package frc.robot.auto;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class MoveOutLeft extends MoveOutMid {
    public MoveOutLeft(DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) {
        super(0.15, 0.3, 1.2, driveTrain, shooter, arm, intake, 1.46);
    }
    
    @Override
    public String getName() {
        return "Move Out Left";
    }
}
