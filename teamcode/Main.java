package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// main op mode
@TeleOp
public class Main extends OpMode {
    // handlers
    DrivingHandler    drivingHandler;

    // initialization
    public void init() {
        // setup handlers
        drivingHandler    = new DrivingHandler(hardwareMap);

        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    // gameplay loop
    public void loop() {
        // run handlers
        drivingHandler.loop(gamepad1);

        telemetry.addData("status", "running");
        telemetry.update();
    }
}
