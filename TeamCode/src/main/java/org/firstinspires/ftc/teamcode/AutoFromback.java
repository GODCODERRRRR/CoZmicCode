package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AutoFromback extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException{

       DcMotorEx topLeftMotor = hardwareMap.get(DcMotorEx.class,"topLeft");
        DcMotorEx topRightMotor = hardwareMap.get(DcMotorEx.class,    "topRight");
       DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"backLeft");
       DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"backRight");

        topRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if(isStopRequested()) return;

        if(opModeIsActive()){
                topLeftMotor.setPower(0.5);
                topRightMotor.setPower(0.5);
                backLeftMotor.setPower(0.5);
                backRightMotor.setPower(0.5);

                sleep(2000);

                topLeftMotor.setPower(0);
                topRightMotor.setPower(0);
                backLeftMotor.setPower(0);
                backRightMotor.setPower(0);
        }
    }
}
