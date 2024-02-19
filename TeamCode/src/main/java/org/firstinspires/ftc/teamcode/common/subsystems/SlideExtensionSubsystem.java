package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlideExtensionSubsystem extends SubsystemBase {
    private final DcMotor slideExtension;
    private final int slideBackboard = 1600;
    private final int slideExtended = 1800;
    private final int slideStartPos = 10;

    public SlideExtensionSubsystem(final HardwareMap hMap, final String name) {
        slideExtension = hMap.get(DcMotor.class, name);
    }

    public void setSlideStartPos() {
        slideExtension.setTargetPosition(slideStartPos);
    }
    public void setSlideBackboard() {
        slideExtension.setTargetPosition(slideBackboard);
    }
    public void setSlideExtended() {
        slideExtension.setTargetPosition(slideExtended);
    }

}
