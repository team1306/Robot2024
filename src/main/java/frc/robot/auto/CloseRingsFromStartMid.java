package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.NoteDetector;

public class CloseRingsFromStartMid extends SequentialCommandGroup {
    public CloseRingsFromStartMid(NoteDetector detector, Intake intake, Shooter shooter, Arm arm) {

        System.out.println("Running Auto");

        addCommands(
            //shoot
            AutoCommands.getClose1StartMid(intake, shooter), //collect close 1 and shoot
            AutoCommands.getClose2Close1(intake, shooter), //collect close 2 and shoot
            AutoCommands.getClose3Close2(intake, shooter) //collect close 3 and shoot
        );
    }
}
