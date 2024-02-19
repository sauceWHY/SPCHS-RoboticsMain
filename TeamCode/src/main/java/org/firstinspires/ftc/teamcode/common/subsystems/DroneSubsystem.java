package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneSubsystem extends SubsystemBase {
    private final Servo drone;
    private double droneSecured = 0.2;
    private double droneByeBye = 1;

    public DroneSubsystem(final HardwareMap hMap, final String name) {
        drone = hMap.get(Servo.class, name);
    }

    public void setDroneSecured() {
        drone.setPosition(droneSecured);
    }
    public void setDroneByeBye() {
        drone.setPosition(droneByeBye);
    }


}
