package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="Auto settings")
public class AutoSettings extends LinearOpMode {
    // This is a linear opmode, which has a different format than the standard opMode.
    public void runOpMode() {
        while (opModeInInit()) { // While we are in init, you can change the settings
            // I feel like the rest of this is pretty self explanatory
            if (gamepad1.xWasPressed()) {
                if (Storage.color == 1) {
                    Storage.color = -1;
                } else {
                    Storage.color = 1;
                }
            }

            if (gamepad1.aWasPressed()) {Storage.close = !Storage.close;}
            if (gamepad1.yWasPressed()) {Storage.parkClose = !Storage.parkClose;}
            if (gamepad1.dpadUpWasPressed()) {Storage.MSWaitForTip += 1000;}
            if (gamepad1.dpadDownWasPressed()) {Storage.MSWaitForTip -= 1000;}


            telemetry.addLine("Color is currently " + (Storage.color == 1 ? "blue" : "red") + ".");
            telemetry.addLine("         Press x to toggle");
            telemetry.addLine("Set auto is currently " + (Storage.close ? "on garden side" : "not garden side") + ".");
            telemetry.addLine("         Press a to toggle");
            telemetry.addLine("Set park is currently " + (Storage.parkClose ? "on garden side" : "not garden side") + ".");
            telemetry.addLine("         Press y to toggle");
            telemetry.addLine("Wait is currently " + Storage.MSWaitForTip + " milliseconds" + ".");
            telemetry.addLine("         Control with dPad");

            telemetry.update();
        }
    }
}
