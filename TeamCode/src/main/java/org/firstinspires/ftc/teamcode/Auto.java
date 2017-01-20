package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Winston on 11/14/2015.
 */
@Autonomous(name="Auto", group="Auto")
public class Auto extends OpMode {

    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;

    public void init()
    {
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");

        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop() {

        double left, right;

        if(this.time <= 8) {
            left = .3;
            right = .3;
        } else {
            left = 0;
            right = 0;
        }


		/*
		 * Set the motor power
		 */
        frontRightMotor.setPower(right);
        backRightMotor.setPower(right);
        frontLeftMotor.setPower(left);
        backLeftMotor.setPower(left);
    }

    public void stop() {

    }

}
