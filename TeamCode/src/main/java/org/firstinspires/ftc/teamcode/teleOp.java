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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;

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

@TeleOp(name="teleOp")
public class teleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor left1;
    public DcMotor left2;
    public DcMotor right1;
    public DcMotor right2;
    public DcMotor rightLift;
    public DcMotor leftLift;
    public DcMotor leftIntake;
    public DcMotor rightIntake;
    public CRServo frontAdjust;
    public CRServo metalPlate1;
    public CRServo metalPlate2;
    public ColorSensor identifyJewel;
    public CRServo knockerBase;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left1 = hardwareMap.dcMotor.get("leftFront");
        left2 = hardwareMap.dcMotor.get("leftRear");
        right1 = hardwareMap.dcMotor.get("rightFront");
        right2 = hardwareMap.dcMotor.get("rightRear");
        rightLift = hardwareMap.dcMotor.get("rightSlide");
        leftLift = hardwareMap.dcMotor.get("leftLift");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        frontAdjust = hardwareMap.crservo.get("frontAdjust");
        metalPlate1 = hardwareMap.crservo.get("metalPlate1");
        metalPlate2 = hardwareMap.crservo.get("metalPlate2");
        identifyJewel = hardwareMap.colorSensor.get("identifyJewel");
        knockerBase = hardwareMap.crservo.get("knockerBase");
//        identifyJewel = hardwareMap.SensorREVColorDistance.get("identifyJewel");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);
        left1.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.FORWARD);
        //Set motors to brake
        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Set motor power variables
        double leftFront;
        double leftBack;
        double rightFront;
        double rightBack;
        //Set directional drive variables
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        //Send power commands to motors
        leftFront = drive+strafe+turn;
        leftBack = drive-strafe+turn;
        rightFront = drive-strafe-turn;
        rightBack = drive+strafe-turn;

        // Send calculated power to wheels
        left1.setPower(leftFront);
        left2.setPower(leftBack);
        right1.setPower(rightFront);
        right2.setPower(rightBack);

        //Run intake motors
        if (gamepad2.b){
            leftIntake.setPower(1);
            rightIntake.setPower(-1);
        }else if (gamepad2.a){
            leftIntake.setPower(-1);
            rightIntake.setPower(1);
        }else{
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }

        //Tilt metal platform to deposit glyphs
        if(gamepad2.x){
            metalPlate2.setPower(-.25);
            metalPlate1.setPower(20.0);
        }else if(gamepad2.y){
            metalPlate2.setPower(.60);
            metalPlate1.setPower(.30);
        } else if (gamepad2.right_bumper){
            metalPlate1.setPower(.8);
            metalPlate2.setPower(.1);
        }
        if (gamepad2.left_bumper){
            frontAdjust.setPower(1);
        }else{
            frontAdjust.setPower(0);
        }

        if (gamepad2.dpad_up){
            rightLift.setPower(-.5);
            leftLift.setPower(.5);
        }else if(gamepad2.dpad_down){
            rightLift.setPower(.5);
            leftLift.setPower(-.5);
        }else{
            rightLift.setPower(0);
            leftLift.setPower(0);
        }


        knockerBase.setPower(0);


//        String colorInFront = "";

//        if(identifyJewel.red() > 100) {
//            colorInFront = "red";
//        }else if (identifyJewel.blue()>100){
//            colorInFront = "blue";
//        }else{
//            colorInFront = "no color detected";
//        }

        // Telemetry data
        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("RED: ", identifyJewel.red());
//        telemetry.addData("BLUE: ", identifyJewel.blue());
//        telemetry.addData("GREEN: ", identifyJewel.green());
//        telemetry.addData("Hue: ", identifyJewel.argb());
//        telemetry.addData("Color Identified:", colorInFront);
//        telemetry.addData("Servo Position: ", knockerBase.getPower());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
