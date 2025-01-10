package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/*
config stuff:
- motor arm is a dc motor under "motorArm"
- the grabbing servo is under "grabServo"
 */

// main op mode
@TeleOp
public class Main extends OpMode {
    // handlers
    DrivingHandler    drivingHandler;
    GrabServoHandler  grabServoHandler;

    // initialization
    public void init() {
        // setup handlers
        drivingHandler    = new DrivingHandler(hardwareMap);
        grabServoHandler  = new GrabServoHandler(hardwareMap);
        telemetry.addData("status", "initialized");
        telemetry.update();
    }

    // gameplay loop
    public void loop() {
        // run handlers
        drivingHandler.loop(gamepad1, telemetry);
        grabServoHandler.loop(gamepad1);
        telemetry.addData("status", "running");
        telemetry.update();
    }
}