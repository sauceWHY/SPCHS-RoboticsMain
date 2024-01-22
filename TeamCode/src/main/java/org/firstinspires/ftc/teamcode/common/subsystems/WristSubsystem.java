package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristSubsystem extends SubsystemBase {
    private final Servo wrist;

    /*
    I've heard abt ways to get around having to make new instances every time
    a subsystem is processed; is that referencing mapping it to the Servo object
    above and bellow as the problem? If so what is the problem, threads? perchance.
     */
    public WristSubsystem(final HardwareMap hMap, final String name) {
        wrist = hMap.get(Servo.class, name);
    }
    //could make a function that adjusts servo position relative to arm angle
    public void board() {
        
    }
}
