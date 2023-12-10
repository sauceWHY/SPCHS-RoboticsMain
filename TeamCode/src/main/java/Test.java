import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RR.drive.trajectorysequence.TrajectorySequence;

@Autonomous(name = "test", group = "testing")
public class Test extends LinearOpMode {

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence Test = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .forward(72)
                .strafeRight( 36)
                .build();

        waitForStart();
        drive.followTrajectorySequence(Test);
        if (isStopRequested()) return;

    }
}