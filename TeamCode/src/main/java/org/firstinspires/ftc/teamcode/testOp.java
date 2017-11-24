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


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@Autonomous(name = "testOp")
public class testOp extends LinearOpMode{
    DcMotor leftDrive1;
    DcMotor leftDrive2;
    DcMotor rightDrive1;
    DcMotor rightDrive2;
    DcMotor armBase;

    Servo armJoint;
    Servo leftClamp;
    Servo rightClamp;

    @Override
    public void runOpMode() throws InterruptedException{
        leftDrive1 = hardwareMap.dcMotor.get("drive1");
        leftDrive2 = hardwareMap.dcMotor.get("drive2");
        rightDrive1 = hardwareMap.dcMotor.get("drive3");
        rightDrive2 = hardwareMap.dcMotor.get("drive4");
        armBase = hardwareMap.dcMotor.get("armBase");

        armJoint = hardwareMap.servo.get("armJoint");
        leftClamp = hardwareMap.servo.get("clamp1");
        rightClamp = hardwareMap.servo.get("clamp2");

        waitForStart();

        leftDrive1.setPower(1.0);
        leftDrive2.setPower(1.0);
        rightDrive1.setPower(-1.0);
        rightDrive2.setPower(-1.0);

        leftClamp.setPosition(0);
        rightClamp.setPosition(1.0);

        ElapsedTime eTime = new ElapsedTime();

        eTime.reset();

        while(eTime.time()< 2.5){}

        leftDrive1.setPower(0);
        leftDrive2.setPower(0);
        rightDrive1.setPower(0);
        rightDrive2.setPower(0);

        leftClamp.setPosition(1.0);
        rightClamp.setPosition(0);

        eTime.reset();

        while(eTime.time()< 2.5){}
    }
}


