package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.NoteDetector;

public class CloseRingsFromStartMid extends SequentialCommandGroup {
    public CloseRingsFromStartMid(NoteDetector detector, Intake intake, Shooter shooter, Arm arm) {
        System.out.println();
        System.out.println("Running Commands");
        System.out.println();

        addCommands(
            //shoot
            AutoCommands.getClose1StartMid(intake), //collect close 1 and shoot
            AutoCommands.getClose2Close1(intake), //collect close 2 and shoot
            AutoCommands.getClose3Close2(intake) //collect close 3 and shoot
        );
    }
}
