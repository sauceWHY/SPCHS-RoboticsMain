package Competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static Competition.magic.armmotor;
import static Competition.magic.hand;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;




import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoRedBoardStart", group = "Competition")
public class AutoRedBoardStart extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        /* Trajectories either consist of vectors or poses. Vectors are for moving only x and y coordinates while poses have a heading(angle)
            For example
            a pose at coordinates (10,-10) facing 120 degrees would look like
            Pose2d myPose = new Pose2d(10,-10, Math.toRadians(120));
            Assuming you start at (0,0) at the start of the program, the robot with move to the coordinates labeled at an 120 degree heading
         */
        TrajectorySequence genesis = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(270))) //(0,0) is the starting position and 270 degrees is the direction it is facing if you put it on a coordinate system(straight down)
                        .splineToLinearHeading(new Pose2d(25,25,Math.toRadians(0)), Math.toRadians(270))
                        .addTemporalMarker(() -> {
                            armmotor.setPower(.50);
                            hand.setPosition(0); // lowers servo
                        })
                        .waitSeconds(3)
                        .build();


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(genesis);

    }
}