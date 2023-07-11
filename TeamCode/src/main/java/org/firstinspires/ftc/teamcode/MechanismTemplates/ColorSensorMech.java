package org.firstinspires.ftc.teamcode.MechanismTemplates;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorMech {
    private ColorSensor color;
    private String name;

    public ColorSensorMech(HardwareMap hardwareMap, String name) {
        this.name = name;
        color = hardwareMap.get(ColorSensor.class, name);
    }

    public boolean isDetectingRed(){
        if ((color.red() > color.green()) && (color.red() > color.blue()))
            return true;
        return false;
    }
    public void update(Telemetry telemetry){
        telemetry.addData(name + "Red", color.red());
        telemetry.addData(name + "Green", color.green());
        telemetry.addData(name +"Blue", color.blue());
        //telemetry.addData("Alpha", color.alpha());
        telemetry.update();
    }

}
