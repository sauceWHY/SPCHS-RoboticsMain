package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RiggingArmSubsystem extends SubsystemBase {
    private final DcMotor riggingArm;
    private final int rigArmDown = 0;
    private final int rigArmUp = 3900;
    private final int rigArmDronePos = 2500;

    public RiggingArmSubsystem(final HardwareMap hMap, final String name) {
        riggingArm = hMap.get(DcMotor.class, name);
    }

    public void setRigArmDown() {
        riggingArm.setTargetPosition(rigArmDown);
    }
    public void setRigArmUp() {
        riggingArm.setTargetPosition(rigArmUp);
    }
    public void setRigArmDronePos() {
        riggingArm.setTargetPosition(rigArmDronePos);
    }
}
