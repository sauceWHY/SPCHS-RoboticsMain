package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LeftClawSubsystem extends SubsystemBase {
    private final Servo leftClaw;
    private double closedClaw = 0.17;
    private double openedClaw = 0.54;

    public LeftClawSubsystem(final HardwareMap hMap, final String name) {
        leftClaw = hMap.get(Servo.class, name);
    }

    public void close() {
        leftClaw.setPosition(closedClaw);
    }

    public void open() {
        leftClaw.setPosition(openedClaw);
    }

    @Override
    public void periodic() {
        /*
        put telemetry update here AFTER deciding where to put state machine enum updates
        im guessing in the methods above but idk fs...
         */
    }
}
