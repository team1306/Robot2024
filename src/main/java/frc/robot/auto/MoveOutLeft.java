package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class MoveOutLeft extends SequentialCommandGroup {
    private DriveTrain driveTrain = new DriveTrain(null);
    public MoveOutLeft(Shooter shooter, Arm arm) {

        System.out.println("Running Auto");

        addCommands(
            new MoveArmToSetpointCommand(arm, Arm.Setpoint.SHOOT_CLOSE),
            //shoot

            new ParallelDeadlineGroup(new WaitCommand(2),
                driveTrain.driveBySetpointPercentagesCommand(0.15,0.3)
            )
        );
    }
}
