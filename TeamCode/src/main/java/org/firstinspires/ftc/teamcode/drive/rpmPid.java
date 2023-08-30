package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Config
public class rpmPid extends LinearOpMode {
    DcMotorEx motor;
    double integralSum =0;
    public static double Kp= 0;
    //public static double Ki= 0;
    public static double Kd= 0;
    public static double Kf =0.000631;
    public double maxVelo = 0;


    ElapsedTime timer = new ElapsedTime();
    private double lastError =0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        while (opModeIsActive()) {
            double power = PIDControl(950, motor.getVelocity()) ;
            motor.setPower(power);
            telemetry.addData("power", power);
            if (motor.getVelocity() > maxVelo)
                maxVelo = motor.getVelocity();

            telemetry.addData("RPM", (motor.getVelocity()/103.8)*60);
            telemetry.addData("error", ((motor.getVelocity()/103.8)*60) - 550 );
            telemetry.addData("max velo", maxVelo);
            telemetry.addData("timer", timer.seconds());
            telemetry.update();

        }
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        //integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd)  + (reference * Kf); //+ (integralSum * Ki));
        return output;
    }
}
