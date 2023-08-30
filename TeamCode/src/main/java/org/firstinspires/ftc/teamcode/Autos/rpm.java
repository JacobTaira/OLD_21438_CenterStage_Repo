package org.firstinspires.ftc.teamcode.Autos;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Config
public class rpm extends LinearOpMode {

        private DcMotorEx motor;
        /*
        public static final double NEW_P = 2.5;
        public static final double NEW_I = 0.1;
        public static final double NEW_D = 0.2;
        public static final double NEW_F = 0.5;

         */
        public static double TARGET_POS = 100;

        public static PIDCoefficients testPID = new PIDCoefficients(0,0,0);
        ElapsedTime PIDTimer = new ElapsedTime();

        double integral = 0;
        double repetitions = 0;


    @Override
        public void runOpMode() {
            // Initialize the motor
            motor = hardwareMap.get(DcMotorEx.class, "motor");


            waitForStart();

           //moveMotor(TARGET_POS);


/*
        PIDFCoefficients pidfOrig = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
        PIDFCoefficients pidfModified = motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

 */



        motor.setPower(0.5);

            // Start the loop that will measure and print the RPM
            while (opModeIsActive()) {
                telemetry.addData("RPM", (motor.getVelocity()/145)*60);
                telemetry.update();
            }
        }


    }

