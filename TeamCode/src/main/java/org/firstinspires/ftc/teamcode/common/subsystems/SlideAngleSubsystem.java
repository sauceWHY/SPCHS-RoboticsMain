package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlideAngleSubsystem extends SubsystemBase {

    private final DcMotor slideAngle;
    private final int boardAngle = 1200;
    private final int pixelArmAngle = 3125;
    private final int armStartPos = 0;
    private final int armRestingPos = 2200;

    public SlideAngleSubsystem(final HardwareMap hMap, final String name) {
        slideAngle = hMap.get(DcMotor.class, name);
    }

    public void setBoardAngle() {
        slideAngle.setTargetPosition(boardAngle);
    }
    public void setPixelArmAngle() {
        slideAngle.setTargetPosition(pixelArmAngle);
    }
    public void setArmStartPos() {
        slideAngle.setTargetPosition(armStartPos);
    }
    public void setArmRestingPos() {
        slideAngle.setTargetPosition(armRestingPos);
    }
}
