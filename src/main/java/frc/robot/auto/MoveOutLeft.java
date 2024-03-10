package frc.robot.auto;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class MoveOutLeft extends MoveOutMid {
    public MoveOutLeft(DriveTrain driveTrain, Shooter shooter, Arm arm, Intake intake) {
        super(0.3, 0.07, driveTrain, shooter, arm, intake);
    }
}
