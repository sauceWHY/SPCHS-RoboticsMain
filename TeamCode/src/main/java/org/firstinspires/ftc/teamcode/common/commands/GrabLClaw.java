package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.subsystems.LeftClawSubsystem;

public class GrabLClaw extends CommandBase {
    private final LeftClawSubsystem lClaw;

    public GrabLClaw(LeftClawSubsystem subsystem) {
        lClaw = subsystem;
        addRequirements(lClaw);
    }

    @Override
    public void initialize() {
        lClaw.open();
        new WaitCommand(500);
        lClaw.close();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
