package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Example Auto")
public class AutoExample extends LinearOpMode {
    // We generally use LinearOpModes for autonomous
    public void runOpMode() {
        // This code will run once when you press init
        // (set your subsystems and motors here)

        while (opModeInInit()) {
            // This code runs repeatedly while the opmode is in init waiting for start
        }

        // This code will run once when you press start

        while (opModeIsActive()) {
            // This code will run repeatedly until someone presses stop
        }

    }

}
