package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.VisualLocalizationPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class VisualLocalizationTest extends LinearOpMode {

    private int width;
    private int height;

    private double focalLenghtPixels;
    private double focalLenght;
    private double sensorHeight;
    private double realHeight;
    private OpenCvWebcam backCam;

    private Pose2d x = new Pose2d();
    private SampleMecanumDrive drive;

    private VisualLocalizationPipeline detector;

    public void initialize() {
        //initializing the basic variables and objects
        width = 160;
        height = 120;
        focalLenghtPixels = 1430;
        focalLenght = 4;
        sensorHeight = 2.02; //possibility of being 3.58
        realHeight = 10;
        drive = new SampleMecanumDrive(hardwareMap);
        detector = new VisualLocalizationPipeline(focalLenghtPixels,focalLenght,sensorHeight,telemetry,realHeight);

        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        backCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamFront"), cameraMonitorViewId);
        backCam.setPipeline(detector);

        backCam.setMillisecondsPermissionTimeout(2500);
        backCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                telemetry.addLine("started");
                backCam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("not open");
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        TrajectorySequence turnToJunction = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(detector.getDistanceX(),detector.getDistanceY(),Math.toRadians(detector.getAngle())))
                .build();

        waitForStart();

            drive.followTrajectorySequence(turnToJunction);


    }
}
