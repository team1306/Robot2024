package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VibrateControllersCommand extends Command {
    public static final double RUMBLE_TIME = .8;
    private final HIDSubsystem[] hids;

    public static class HIDSubsystem extends SubsystemBase {
        public final GenericHID hid;
        public HIDSubsystem(GenericHID hid) {
            this.hid = hid;
        }
    }

    public VibrateControllersCommand(HIDSubsystem... genericHIDs) {
        this.hids = genericHIDs;

        addRequirements(genericHIDs);
    }

    private void setRumble(double val) {
        for (HIDSubsystem hidSubsystem : hids) {
            hidSubsystem.hid.setRumble(RumbleType.kBothRumble, val);
        }
    }

    @Override
    public void initialize() {
        setRumble(1);
    }

    @Override
    public void end(boolean interrupted) {
        setRumble(0);
    }
}
