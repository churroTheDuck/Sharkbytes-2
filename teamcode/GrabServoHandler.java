package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

// grab servo handler
public class GrabServoHandler {
    // grab servo
    Servo   grabServo;
    boolean grabbing;
    boolean justPressed;

    // initialization
    public GrabServoHandler(HardwareMap hardwareMap) {
        // set grab servo
        grabServo = hardwareMap.get(Servo.class, "grabServo");

        // set initial state
        grabbing    = false;
        justPressed = false;
    }

    // gameplay loop
    public void loop(Gamepad gamepad1) {
        if (!justPressed && gamepad1.a) {
            // set grabbing state
            grabbing = !grabbing;
        }

        justPressed = gamepad1.a;

        // set servo position
        grabServo.setPosition(grabbing ? 0.65 : 0.8);
    }
}
