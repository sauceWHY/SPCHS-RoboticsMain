package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RightClawSubsystem extends SubsystemBase {

    private final Servo rightClaw;
    private double closedClaw = 0.6;
    private double openedClaw = 0.3;

    public RightClawSubsystem(final HardwareMap hMap, final String name) {
        rightClaw = hMap.get(Servo.class, name);
    }

    public void close() {
        rightClaw.setPosition(closedClaw);
    }

    public void open() {
        rightClaw.setPosition(openedClaw);
    }

    @Override
    public void periodic() {
        /*
        put telemetry update here AFTER deciding where to put state machine enum updates
        im guessing in the methods above but idk fs...
         */
    }
}
