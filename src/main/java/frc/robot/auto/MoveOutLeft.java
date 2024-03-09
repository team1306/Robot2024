package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.MoveArmToSetpointCommand;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class MoveOutLeft extends SequentialCommandGroup {
    private DriveTrain driveTrain = new DriveTrain(null);
    public MoveOutLeft(Shooter shooter, Arm arm, Intake intake) {

        System.out.println("Running Auto");
        final Command shooterCommand = new ToggleShooterCommand(() -> 1, () -> arm.getCurrentAngle().getDegrees(), shooter);
        addCommands( //all commands run at once
            shooterCommand, //turns on shooter
            new SequentialCommandGroup( //following commands run in sequence
                new MoveArmToSetpointCommand(arm, Arm.Setpoint.SHOOT_CLOSE), //aim
                new WaitCommand(2),
                new IntakeIndexCommand(intake), //fire
                new InstantCommand(() -> shooterCommand.cancel()), //turn off shooter
                new MoveArmToSetpointCommand(arm, Arm.Setpoint.DOWN), //arm down
                new ParallelDeadlineGroup(new WaitCommand(2), //drive out
                    driveTrain.driveBySetpointPercentagesCommand(0.3,0.15)
                )
            )
        );
    }
}
