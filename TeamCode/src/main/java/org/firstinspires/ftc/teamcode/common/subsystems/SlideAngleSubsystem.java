package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SlideAngleSubsystem extends SubsystemBase {

    private final DcMotor slideAngle;
    private final int boardAngle = 2400;
    private final int pixelArmAngle = 3125;
    private final int armRestingPosition = 0;
    private final int armUnderBar = 2200;

    public SlideAngleSubsystem(final HardwareMap hMap, final String name) {
        slideAngle = hMap.get(DcMotor.class, name);
    }

    public void setBoardAngle() {
        slideAngle.setTargetPosition(boardAngle);
    }
    public void setPixelArmAngle() {
        slideAngle.setTargetPosition(pixelArmAngle);
    }
    public void setArmRestingPosition() {
        slideAngle.setTargetPosition(armRestingPosition);
    }
    public void setArmUnderBar() {
        slideAngle.setTargetPosition(armUnderBar);
    }
}
