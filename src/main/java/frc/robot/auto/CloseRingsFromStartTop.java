package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intake.IntakeIndexCommand;
import frc.robot.commands.shooter.ToggleShooterCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.NoteDetector;

public class CloseRingsFromStartTop extends SequentialCommandGroup {
    public CloseRingsFromStartTop(NoteDetector detector, Intake intake, Shooter shooter, Arm arm) {

        System.out.println("Running Auto");
        final Command shooterCommand = new ToggleShooterCommand(() -> .79, Arm.SetpointOptions.SHOOT_CLOSE::getPos, shooter);
           addCommands(
            shooterCommand, //spin up shooter
            new IntakeIndexCommand(intake), //fire
            AutoCommands.getClose1StartTop(intake, shooter), //collect close 1 and shoot
            AutoCommands.getClose2Close1(intake, shooter), //collect close 2 and shoot
            AutoCommands.getClose3Close2(intake, shooter), //collect close 3 and shoot
            new InstantCommand(shooterCommand::cancel) //shooter off

        );
    }
}