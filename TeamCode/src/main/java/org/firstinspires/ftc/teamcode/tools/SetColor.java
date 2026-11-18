package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Set Color")
public class SetColor extends OpMode {
    public void init() {
        telemetry.addLine("Set color is currently " + ((Storage.color == 1) ? "blue" : "red") + ".");
        telemetry.addLine( "Press start if you want to swap the color.");
    }

    public void loop() {
        if (Storage.color == 1) {
            Storage.color = -1;
        } else {
            Storage.color = 1;
        }

        requestOpModeStop();
    };
}
