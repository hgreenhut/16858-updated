package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class Tele extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor xRailMotor;
    private DcMotor carMotor;

    private Servo turnServo;
    private Servo clampServo;
    private double turnTarget;
    private double clampTarget;

    private final double MAX_CAR_POWER = .5;

    private DcMotor[] motors;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // DRIVING
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        //frontLeft.setDirection(DcMotor.Direction.FORWARD);
        //frontRight.setDirection(DcMotor.Direction.REVERSE);
        //backLeft.setDirection(DcMotor.Direction.FORWARD);
        //backRight.setDirection(DcMotor.Direction.REVERSE);

        // INIT MOTORS
        this.motors = new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
        for (int i = 0; i < this.motors.length; i++) {
            DcMotor motor = this.motors[i];
            DcMotor.Direction direction = i % 2 == 0 ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD;

            motor.setDirection(direction);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // XRail
        this.xRailMotor = hardwareMap.get(DcMotor.class, "xr");
        this.carMotor = hardwareMap.get(DcMotor.class, "cm");

        // Servos
        this.turnServo = hardwareMap.get(Servo.class, "ts");
        this.clampServo = hardwareMap.get(Servo.class, "cs");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        this.turnTarget = 0.5; //this.turnServo.getPosition();
        this.clampTarget = 0.5; //this.clampServo.getPosition();

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // DRIVING
        double drive = gamepad1.left_stick_y;

        double turn = gamepad1.right_stick_x;


        double[] powers = {

                drive - turn,
                drive + turn,
                drive - turn,
                drive + turn
        };


        for (int i = 0; i < this.motors.length; i++) {
            this.motors[i].setPower(powers[i]);
            telemetry.addData("Power ", "Run Time: " + runtime.toString());
        }

        // XRAIL
        float upSpeed = gamepad2.right_trigger;
        float downSpeed = gamepad2.left_trigger;
        float totalSpeed = upSpeed - downSpeed;

        this.xRailMotor.setPower(totalSpeed);

        // CAROUSEL MOTOR
        if (this.gamepad2.x) {
            this.carMotor.setPower(MAX_CAR_POWER);
        } else if (this.gamepad2.y) {
            this.carMotor.setPower(-MAX_CAR_POWER);
        } else {
            this.carMotor.setPower(0);
        }


        // SERVO

        if (this.gamepad2.left_bumper) {
            this.turnTarget -= .001;
        }
        else if (this.gamepad2.right_bumper) {
            this.turnTarget += .001;
        }


        if (this.gamepad2.a) {
            this.clampTarget = 1;
        } else if (this.gamepad2.b) {
            this.clampTarget = 0;
        } else {
            this.clampTarget = .5;
        }

        this.clampServo.setPosition(this.clampTarget);
        this.turnServo.setPosition(this.turnTarget);


        // DATA
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Clamp Target", this.clampTarget);
        telemetry.addData("Clamp Position", this.clampServo.getPosition());
        telemetry.addData("Turn Target", this.turnTarget);
        telemetry.addData("Turn Position", this.turnServo.getPosition());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        for (DcMotor motor : this.motors) {
            motor.setPower(0);
        }

        this.xRailMotor.setPower(0);
        this.carMotor.setPower(0);
    }

}
