package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous
public class Autonforred extends OpMode {
    enum State { // Steps
        Start,
        SecondStep,
        ThirdStep,
        Cuatro,
        FifthStep,
        SixSeven,
        Reset,
        GoToLoadingZone,
        AlmostDone,
        Turning,
        Finally,
        Done
    }
    double a = 0.5;
    double b = 0;
    double c = 1.0; //To Organize, Requires less work to change one thing than more than one
    MechanismsRobo drive = new MechanismsRobo();
    MechanismsRobo board = new MechanismsRobo();
    State state = State.Start;
    double lastTime;
    @Override
    public void init() {
        drive.init(hardwareMap);
        board.init(hardwareMap);
    }
    @Override
    public void start(){
        state = State.Start;
        resetRuntime();
        lastTime = getRuntime();
    }
    @Override
    public void loop() {
        telemetry.addData("State", state);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Time in State", getRuntime()- lastTime); //to stay organized.
        switch (state) {
            case Start:
                if (getRuntime() >= 1.0) {
                    board.setMotorSpeed(0.65); //Pretty sure this is the right speed, don't change unless it doesn't work
                    lastTime = getRuntime();
                    state = State.SecondStep;
                }
                break;
               /*  Will be repeating second step because this is
               * what gets the servos moving to shoot the balls
                each time we have to move the servos forward then backward*/
            case SecondStep:
                if (getRuntime() >= lastTime + 4.0) {
                    board.setServoRotRight(a);
                    board.setServoRotLeft(-a);
                    lastTime = getRuntime();
                    state = State.ThirdStep;
                }
                break;
            case ThirdStep:
                if (getRuntime() >= lastTime + c) {
                    board.setServoRotRight(b);
                    board.setServoRotLeft(b);
                    lastTime = getRuntime();
                    state = State.Cuatro;
                }
                break;
            case Cuatro:
                if (getRuntime() >= lastTime + c) {
                    board.setServoRotRight(a);
                    board.setServoRotLeft(-a);
                    lastTime = getRuntime();
                    state = State.FifthStep;
                }
                break;
            case FifthStep:
                if (getRuntime() >= lastTime + c) {
                    board.setServoRotRight(b);
                    board.setServoRotLeft(b);
                    lastTime = getRuntime();
                    state = State.SixSeven;
                }
                break;
            case SixSeven:
                if (getRuntime() >= lastTime + c) {
                    board.setServoRotRight(a);
                    board.setServoRotLeft(-a);
                    lastTime = getRuntime();
                    state = State.Reset;
                }
                break;
            // Need to reset everything, so made a reset step
            case Reset:
                if (getRuntime() >= lastTime + 2) {
                    board.setMotorSpeed(0);
                    board.setServoRotLeft(b);
                    board.setServoRotRight(b);
                    lastTime = getRuntime();
                    state = State.GoToLoadingZone;
                }
                break;
            // Have to make it easier for the driver, so before teleop starts, I will make it move back to loading zone
            case GoToLoadingZone:
                if (getRuntime() >= lastTime + 1) {
                    drive.drive(1, 0, 0);
                    lastTime = getRuntime();
                    state = State.AlmostDone;
                }
                break;
            case AlmostDone:
                if (getRuntime() >= lastTime + 1.867){ // this is to get the robot to loading zone
                    drive.drive(0,0,0);
                    lastTime = getRuntime();
                    state = State.Turning;
                }
                break;
            case Turning:
                if (getRuntime() >= lastTime + c){// same thing as above
                    drive.drive(0,-1,0);
                    lastTime = getRuntime();
                    state = State.Finally;
                }
                break;
            case Finally:
                if (getRuntime() >= lastTime + 1.15){
                    drive.drive(-1,0,0); //finally ends in loading zone
                    lastTime = getRuntime();
                    state = State.Done;
                }
                break;
            case Done:
                if (getRuntime() >= lastTime + 0.5){
                    drive.drive(0,0,0);
                }
                break;
            default:
                telemetry.addData("Autonomous", "Finished");
        }
    }
}


