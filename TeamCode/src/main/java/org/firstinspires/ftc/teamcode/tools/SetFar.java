package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Set Far")
public class SetFar extends OpMode {
    public void init() {
        telemetry.addLine("Set auto is currently " + ((Storage.close) ? "close" : "far") + ".");
        telemetry.addLine( "Press start if you want to swap the position.");
    }

    public void loop() {
        Storage.close = !Storage.close;

        requestOpModeStop();
    };
}
