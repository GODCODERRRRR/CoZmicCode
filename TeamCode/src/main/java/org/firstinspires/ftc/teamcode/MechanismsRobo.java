package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;








public class MechanismsRobo {
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor Launcher;
    private Servo servoRotRight;
    private Servo servoRotLeft;


    public void init(HardwareMap hwMap){




        topLeftMotor = hwMap.get(DcMotor.class,"topLeft");
        topRightMotor = hwMap.get(DcMotor.class,    "topRight");
        backLeftMotor = hwMap.get(DcMotor.class,"backLeft");
        backRightMotor = hwMap.get(DcMotor.class,"backRight");
        Launcher = hwMap.get(DcMotor.class,"Launching");
        servoRotRight = hwMap.get(Servo.class,"rightMover");
        servoRotLeft = hwMap.get(Servo.class, "leftMover");








        Launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }




    public void setPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower){
        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));




        frontLeftPower /= maxSpeed;
        frontRightPower /= maxSpeed;
        backLeftPower /= maxSpeed;
        backRightPower /= maxSpeed;




        topLeftMotor.setPower(frontLeftPower);
        topRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }
    public void drive(double forward, double right, double rotate){
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;




        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
    public void setMotorSpeed(double speed){
        Launcher.setPower(speed);
    }
    public void setServoRotRight(double position){
        servoRotRight.setPosition(position);
    }
    public void setServoRotLeft(double position){
        servoRotLeft.setPosition(position);
    }
}
