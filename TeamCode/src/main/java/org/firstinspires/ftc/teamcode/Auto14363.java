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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="DriveForward Auto 14363", group="Linear Opmode")
public class Auto14363 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeftDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontRightDrive = null;
    private DistanceSensor yeetSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        yeetSensor = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive  = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        drive(.5);

        /*frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("motor position: ", frontLeftDrive.getCurrentPosition());
        telemetry.update();
        frontRightDrive.setTargetPosition(500);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setTargetPosition(500);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setPower(1);
        frontRightDrive.setPower(1);
        backLeftDrive.setPower(1);
        backRightDrive.setPower(1);

        while (opModeIsActive() && frontRightDrive.isBusy() && frontLeftDrive.isBusy())   //frontLeftDrive.getCurrentPosition() < frontLeftDrive.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left", frontLeftDrive.getCurrentPosition() + "  busy=" + frontLeftDrive.isBusy());
            telemetry.update();
            idle();
        }

        backRightDrive.setTargetPosition(500);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setTargetPosition(500);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setPower(.7);
        frontRightDrive.setPower(.7);
        backLeftDrive.setPower(.7);
        backRightDrive.setPower(.7);

        while (opModeIsActive() && (backRightDrive.isBusy() || backLeftDrive.isBusy()))   //frontLeftDrive.getCurrentPosition() < frontLeftDrive.getTargetPosition())
        {
            if(backLeftDrive.getCurrentPosition() >= 500){
                backLeftDrive.setPower(0);
            }

            if(backRightDrive.getCurrentPosition() >= 500){
                backRightDrive.setPower(0);
            }

            telemetry.addData("back left", backLeftDrive.getCurrentPosition() + "  busy=" + backLeftDrive.isBusy());
            telemetry.addData("back right", backRightDrive.getCurrentPosition() + "  busy=" + backRightDrive.isBusy());
            telemetry.update();

            idle();
        }
        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);*/



        /*drive(.5);
        turnLeft(90);
        drive(.5);*/


        //turnLeft(180);
        //turnRight(90);



        //if (opModeIsActive()) {

            /*if(this.getRuntime() < 3){
                leftPower = 1.0;
                rightPower = 1.0;
            } else {
                leftPower = 0;
                rightPower = 0;
            }*/

            /*setPower(1, 1);
            Thread.sleep(1000);
            setPower(-1, -1);
            Thread.sleep(1000);
            setPower(0,0);
            setPower(1, 1);
            Thread.sleep(1000);
            setPower(-1, -1);
            Thread.sleep(1000);
            setPower(0,0);*/

            // Show the elapsed game time and wheel power.
            /*telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();*/
        }
    //}

    /*public void turnLeft(int angle) throws InterruptedException {
        setPower(-.8, .8);
        Thread.sleep((long)(angle * .7222222)*10);
        setPower(0,0);
        Thread.sleep(200);
    }

    public void turnRight(int angle) throws InterruptedException {
        double startTime = this.getRuntime();
        while(this.getRuntime() < startTime + (angle * .7222222)/100){
            setPower(.8, -.8);
        }
    }
    */


    public void drive(double time) throws InterruptedException {
        setPower(.8, .8);
        Thread.sleep((long)(time*1000));
        setPower(0,0);
        Thread.sleep(200);
    }


    public void setPower(double leftPower, double rightPower) throws InterruptedException {
        frontLeftDrive.setPower(leftPower);
        backLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        backRightDrive.setPower(rightPower);
    }
}
