package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PIDSlide {
    private static PIDController controller;
    public static double p=0.005, i=0, d=0.00015;
    public static double f = 0.08;

    public static int target = 0;
    private static final double ticks_per_rev = 537.6;
    private static DcMotorEx leftSlide;
    public static class START {
        static int slidePos = leftSlide.getCurrentPosition();
        static double pid = controller.calculate(slidePos, 0);
        static double ff = Math.cos(Math.toRadians(0 / ticks_per_rev)) * f;

        public static double power = pid + ff;
    }
    public static class FEX {
        static int slidePos = leftSlide.getCurrentPosition();
        static double pid = controller.calculate(slidePos, 500);
        static double ff = Math.cos(Math.toRadians(500 / ticks_per_rev)) * f;

        public static double power = pid + ff;
    }
    public static class TAPE {
        static int slidePos = leftSlide.getCurrentPosition();
        static double pid = controller.calculate(slidePos, 1200);
        static double ff = Math.cos(Math.toRadians(1200 / ticks_per_rev)) * f;

        public static double power = pid + ff;
    }

}


