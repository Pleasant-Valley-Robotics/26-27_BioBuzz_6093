package org.firstinspires.ftc.teamcode;

import static android.graphics.Color.red;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
@TeleOp(name="Sensor test")
public class SensorTest extends OpMode {
    TouchSensor testSensor;

    public void init() {
        testSensor = hardwareMap.get(TouchSensor.class, "testSensor");
    }

    public void loop() {
        telemetry.addData("Sensor data", testSensor.getValue());

        telemetry.update();
    }
}
