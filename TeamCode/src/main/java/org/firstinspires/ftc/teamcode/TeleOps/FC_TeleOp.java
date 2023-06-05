package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//ngl i expected all of this to already be imported under OpMode but oh well

@TeleOp
public class FC_TeleOp extends OpMode {

    private DcMotorEx[] motors;//Array to contain ]all the motors
    private IMU imu;//Imu to access the gyro
    private final double PRECISIONREDUCTION = 0.39;//Reduction factors for presision
    private final double TURN_PRECESION = 0.65;

    @Override
    public void init() {
        motors = new DcMotorEx[4];

        motors[0] = ((DcMotorEx) hardwareMap.dcMotor.get("FL"));//setting the configurations to the motors
        motors[1] = ((DcMotorEx) hardwareMap.dcMotor.get("BL"));
        motors[2] = ((DcMotorEx) hardwareMap.dcMotor.get("FR"));
        motors[3] = ((DcMotorEx) hardwareMap.dcMotor.get("BR"));

        for (DcMotorEx m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//Setting all motors to run without encoders
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//Setting them all to brake
        }

        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu"); //initializing the imu

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        //TODO have to change the orientation accordingly to the position of the bot
        //Meant to tell the imu the configuration of the control hub, and therefore the bot
    }

    //Used to get the max motor power, used to divide all other motor values and keep everything proportional
    private double getMax(double[] input) {
        double max = Integer.MIN_VALUE;
        for (double value : input) {
            if (Math.abs(value) > max) {
                max = Math.abs(value);
            }
        }
        return max;
    }

    //Reducing the dead zone on the controller
    //TODO change the algorithm later
    public double reducingDeadzone(double x) {
        if (x == 0) {
            return 0;
        } else if (0 < x && 0.25 > x) {
            return 0.25;
        } else if (0 > x && x > -0.25) {
            return -0.25;
        }
        return x;
    }

    @Override
    public void loop() {
        double y = reducingDeadzone(-gamepad1.left_stick_y); // Remember, this is reversed!
        double x = reducingDeadzone(gamepad1.left_stick_x);
        boolean precisionToggle = gamepad1.right_trigger > 0.1;
        double rx = reducingDeadzone(-gamepad1.right_stick_x * 0.75);
        if (precisionToggle) {//checks if precisionToggle is on
            rx *= TURN_PRECESION;
        }

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motors[0].setPower(frontLeftPower);
        motors[1].setPower(backLeftPower);
        motors[2].setPower(frontRightPower);
        motors[4].setPower(backRightPower);


    }
}
