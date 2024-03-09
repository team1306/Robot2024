package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class driveBySetpointPercentageCommand {
    public Command driveBySetpointPercentagesCommand(DriveTrain driveTrain, double leftSpeed, double rightSpeed, double seconds) {
        return new Command(){
            {
                addRequirements(driveTrain);
            }

            @Override
            public void initialize() {
                System.out.println("Starting Drive");
                Timer timer = new Timer();
                timer.restart();
            }

            @Override
            public void execute() {
                DriveTrain.setSidePercentages(leftSpeed, rightSpeed);
                System.out.println("Driving");
            }

            @Override
            public void end(boolean interrupted) {
                DriveTrain.setSidePercentages(0, 0);
                System.out.println("Stop driving");
            }
        }
    }
}
