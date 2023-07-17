package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.DetectJunction;
import org.firstinspires.ftc.teamcode.Pipelines.DetectRed;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class JunctionOpmode extends LinearOpMode {

    private int width;
    private int height;

    private double focalLenght;
    private OpenCvWebcam backCam;

    private Pose2d x = new Pose2d();
    private SampleMecanumDrive drive;

    private DetectJunction detector;

    // Kalman filter variables
    private double prevAngle = 0;  // Previous angle measurement
    private double prevX = 0;      // Previous estimated position
    private double xEst = 0;       // Current estimated position
    private double P = 1;          // Covariance matrix (initially set to 1)

    // Kalman filter process and sensor noise
    private double Q = 0.001;      // Process noise (adjust as needed)
    private double R = 0.01;       // Sensor noise (adjust as needed)

    public void initialize()
    {
        //initializing the basic variables and objects
        width = 160;
        height = 120;
        focalLenght = 1430;
        drive = new SampleMecanumDrive(hardwareMap);
        detector = new DetectJunction(telemetry, focalLenght);

        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        backCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamFront"), cameraMonitorViewId);
        backCam.setPipeline(detector);

        backCam.setMillisecondsPermissionTimeout(2500);
        backCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                telemetry.addLine("started");
                backCam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("not open");
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        TrajectorySequence turnToJunction = drive.trajectorySequenceBuilder(new Pose2d())
                .turn(Math.toRadians(detector.getAngle()))
                .build();

        waitForStart();


        while (opModeIsActive())
        {
            // Get the current angle measurement from the detector
            double angle = detector.getAngle();

            // Kalman filter prediction step
            // Predict the next position based on the previous position and control input (angle)
            double xPred = prevX + Math.toRadians(angle);

            // Kalman filter update step
            // Calculate the Kalman gain
            double K = P / (P + R);

            // Update the estimated position using the Kalman gain and the angle measurement
            xEst = prevX + K * (angle - xPred);

            // Update the covariance matrix (P)
            P = (1 - K) * P + Q;

            // Store the current values for the next iteration
            prevAngle = angle;
            prevX = xEst;


            if (detector.getAngle() != 0)
                drive.followTrajectorySequence(turnToJunction);


            telemetry.addData("Estimated X:", xEst);
            telemetry.update();
        }
    }
}
