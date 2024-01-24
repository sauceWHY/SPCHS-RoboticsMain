package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.subsystems.RightClawSubsystem;

public class GrabRClaw extends CommandBase {
    private final RightClawSubsystem rClaw;

    public GrabRClaw(RightClawSubsystem subsystem) {
        rClaw = subsystem;
        addRequirements(rClaw);
    }

    @Override
    public void initialize() {
        rClaw.open();
        new WaitCommand(500);
        rClaw.close();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
