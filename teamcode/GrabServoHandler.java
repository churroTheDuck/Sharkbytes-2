package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

// grab servo handler
public class GrabServoHandler {
    // grab continuous servo
    private Servo grabServo;
    boolean grabbing;
    boolean justPressed;

    // initialization
    public GrabServoHandler(HardwareMap hardwareMap) {
        // set grab continuous servo
        grabServo = hardwareMap.get(Servo.class, "grabServo");

        // set initial state
        grabbing    = false;
        justPressed = false;
    }

    // gameplay loop
    public void loop(Gamepad gamepad1) {
        float rightStickX = gamepad1.right_stick_x;
        //grabServo.setPower(leftStickY);
        grabServo.setPosition(0.5 * (rightStickX + 1));
        // if (!justPressed && gamepad1.a) {
        //     // set grabbing state
        //     grabbing = !grabbing;
        // }

        // justPressed = gamepad1.a;

        // // set servo power for continuous servo
        // if (grabbing) {
        //     grabServo.setPower(1.0);
        // } else {
        //     grabServo.setPower(-1.0);
        // }
    }
}

