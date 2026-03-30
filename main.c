#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRCS.h>
#include <cmath>
#include <FEHServo.h>
#define ECPI 33.741 //encoder counts per inch
#define Pconstant 0.75
#define Iconstant 0.2
#define Dconstant 0.25
#define Deadband 0
#define CdsRedMax 2.0
#define CdsRedMin 0
#define CdsRedMax1 1.4
#define CdsRedMin1 0
#define CdsBlueMax 3.2
#define CdsBlueMin 1.4
#define TurnConstant 24.7
#define LowArmDegree 135
#define HighArmDegree 113
//Pin Declarations
DigitalEncoder right_encoder(FEHIO::P1_0);
DigitalEncoder left_encoder(FEHIO::P2_0);
FEHMotor right_motor(FEHMotor::Motor0,9.0);
FEHMotor left_motor(FEHMotor::Motor1,9.0);
AnalogInputPin CdsCell(FEHIO::P1_3);
AnalogInputPin LeftOptosensor(FEHIO::P1_0);
AnalogInputPin MiddleOptosensor(FEHIO::P1_2);
AnalogInputPin RightOptosensor(FEHIO::P1_4);
FEHServo BigArm(FEHServo::Servo1);

struct PidControl {
float PastError;
int PastCounts;
float PastTime;
float ErrorSum;
};
float LeftPidAdjustment(float ExpectedVelocity, PidControl *LeftMotor) //Inches
per second
{
int CurrentEncoderCount = left_encoder.Counts();
float CurrentTime = TimeNow();
int DeltaCounts = CurrentEncoderCount-LeftMotor->PastCounts; //Find change in
counts since last time
float DeltaTime = CurrentTime - LeftMotor->PastTime; //Find change in time
since last time
if (DeltaTime < 0.001) {
DeltaTime = 0.001;
}
float ActualVelocity = (1.0/ECPI) * (DeltaCounts) / (DeltaTime); //Find
actual velocity
float Error = ExpectedVelocity-ActualVelocity; //Find error
if ((Error > 0 && Error < Deadband) || (Error < 0 && (Error * -1) <
Deadband)){
Error = 0;
}
LeftMotor->ErrorSum = LeftMotor->ErrorSum + Error; //Add error to error sum
float Pterm = Error * Pconstant;
float Iterm = LeftMotor->ErrorSum * Iconstant;
float Dterm = (Error - LeftMotor->PastError) * Dconstant;
LeftMotor->PastError = Error;
LeftMotor->PastCounts = CurrentEncoderCount;
LeftMotor->PastTime = CurrentTime;
return ((ExpectedVelocity + Pterm + Iterm + Dterm) / 17.12168 * 100);
//17.12168 is the max velocity of robot at 100% motor power
}

float RightPidAdjustment(float ExpectedVelocity, PidControl *RightMotor) //Inches
per second
{
int CurrentEncoderCount = right_encoder.Counts();
float CurrentTime = TimeNow();
int DeltaCounts = CurrentEncoderCount-RightMotor->PastCounts; //Find change
in counts since last time
float DeltaTime = CurrentTime - RightMotor->PastTime; //Find change in time
since last time
if (DeltaTime < 0.001) {
DeltaTime = 0.001;
}
float ActualVelocity = (1.0/ECPI) * (DeltaCounts) / (DeltaTime); //Find
actual velocity
float Error = ExpectedVelocity-ActualVelocity; //Find error
if ((Error > 0 && Error < Deadband) || (Error < 0 && (Error * -1) <
Deadband)){
Error = 0;
}
RightMotor->ErrorSum = RightMotor->ErrorSum + Error; //Add error to error sum
float Pterm = Error * Pconstant;
float Iterm = RightMotor->ErrorSum * Iconstant;
float Dterm = (Error - RightMotor->PastError) * Dconstant;
RightMotor->PastError = Error;
RightMotor->PastCounts = CurrentEncoderCount;
RightMotor->PastTime = CurrentTime;
return ((ExpectedVelocity + Pterm + Iterm + Dterm) / 17.12168 * 100);
//17.12168 is the max velocity of robot at 100% motor power
}
void ResetPidVariables(PidControl *LeftMotor, PidControl *RightMotor)
{
LeftMotor->PastError=0;
LeftMotor->PastCounts=0;
LeftMotor->ErrorSum=0;
LeftMotor->PastTime=TimeNow();
RightMotor->PastError=0;
RightMotor->PastCounts=0;
RightMotor->ErrorSum=0;
RightMotor->PastTime=TimeNow();

left_encoder.ResetCounts();
right_encoder.ResetCounts();
Sleep(0.05);
}
void ForwardDrive(float MaxSpeed, float distance, PidControl *LeftMotor,
PidControl *RightMotor)
{
ResetPidVariables(LeftMotor,RightMotor);
float CurrentSpeed = (1.0);
float RampRate = (1.28);
while(((left_encoder.Counts() + right_encoder.Counts()) / 2.0) / ECPI <
distance){
if (CurrentSpeed < MaxSpeed){
CurrentSpeed *= RampRate;
}
if (CurrentSpeed > MaxSpeed){
CurrentSpeed = MaxSpeed;
}
LCD.Clear();
float LeftPower = LeftPidAdjustment(CurrentSpeed,LeftMotor);
float RightPower = RightPidAdjustment(CurrentSpeed,RightMotor);
left_motor.SetPercent(1 * LeftPower);
right_motor.SetPercent(-1 *(RightPower));
LCD.WriteLine("Left Encoder");
LCD.WriteLine(left_encoder.Counts());
LCD.WriteLine("Right Encoder");
LCD.WriteLine(right_encoder.Counts());
Sleep(0.01);
}
//Turn off motors
right_motor.Stop();
left_motor.Stop();
}

void RightTurn(float angle, float speed, PidControl *LeftMotor, PidControl
*RightMotor)
{
ResetPidVariables(LeftMotor,RightMotor);
Sleep(0.05);
while(((left_encoder.Counts() + right_encoder.Counts()) / 2.0) / ECPI <
(TurnConstant/360*angle)){
LCD.Clear();
float LeftPower = LeftPidAdjustment(speed,LeftMotor);
float RightPower = RightPidAdjustment(speed,RightMotor);
left_motor.SetPercent(1 * LeftPower);
right_motor.SetPercent(1 *(RightPower));
LCD.WriteLine(left_encoder.Counts());
LCD.WriteLine(right_encoder.Counts());
Sleep(0.01);
}
//Turn off motors
right_motor.Stop();
left_motor.Stop();
}
void RightArcTurn(float angle, float speed, PidControl *LeftMotor, PidControl
*RightMotor)
{
ResetPidVariables(LeftMotor,RightMotor);
while(((left_encoder.Counts())/2) / ECPI < (TurnConstant/360*angle)){
LCD.Clear();
float LeftPower = LeftPidAdjustment(speed,LeftMotor);
float RightPower = RightPidAdjustment(speed,RightMotor);
left_motor.SetPercent(1 * LeftPower);
right_motor.SetPercent(0 *(RightPower));
LCD.WriteLine(left_encoder.Counts());
LCD.WriteLine(right_encoder.Counts());
Sleep(0.01);
}
//Turn off motors
right_motor.Stop();

left_motor.Stop();
}
void RightBackArcTurn(float angle, float speed, PidControl *LeftMotor, PidControl
*RightMotor)
{
ResetPidVariables(LeftMotor,RightMotor);
while(((left_encoder.Counts()) / 2.0) / ECPI < (TurnConstant/360*angle)){
LCD.Clear();
float LeftPower = LeftPidAdjustment(speed,LeftMotor);
float RightPower = RightPidAdjustment(speed,RightMotor);
left_motor.SetPercent(-1 * LeftPower);
right_motor.SetPercent(0 *(RightPower));
LCD.WriteLine(left_encoder.Counts());
LCD.WriteLine(right_encoder.Counts());
Sleep(0.01);
}
//Turn off motors
right_motor.Stop();
left_motor.Stop();
}
void LeftTurn(float angle, float speed, PidControl *LeftMotor, PidControl
*RightMotor)
{
ResetPidVariables(LeftMotor,RightMotor);
Sleep(0.05);
while(((left_encoder.Counts() + right_encoder.Counts()) / 2.0) / ECPI <
(TurnConstant/360*angle)){
LCD.Clear();
float LeftPower = LeftPidAdjustment(speed,LeftMotor);
float RightPower = RightPidAdjustment(speed,RightMotor);
left_motor.SetPercent(-1 * LeftPower);
right_motor.SetPercent(-1 *(RightPower));
LCD.WriteLine(left_encoder.Counts());
LCD.WriteLine(right_encoder.Counts());
Sleep(0.01);
}

//Turn off motors
right_motor.Stop();
left_motor.Stop();
}
void LeftArcTurn(float angle, float speed, PidControl *LeftMotor, PidControl
*RightMotor)
{
ResetPidVariables(LeftMotor,RightMotor);
while(((right_encoder.Counts())/2) / ECPI < (TurnConstant/360*angle)){
LCD.Clear();
float LeftPower = LeftPidAdjustment(speed,LeftMotor);
float RightPower = RightPidAdjustment(speed,RightMotor);
left_motor.SetPercent(0 * LeftPower);
right_motor.SetPercent(-1 *(RightPower));
LCD.WriteLine(left_encoder.Counts());
LCD.WriteLine(right_encoder.Counts());
Sleep(0.01);
}
//Turn off motors
right_motor.Stop();
left_motor.Stop();
}
void LeftBackArcTurn(float angle, float speed, PidControl *LeftMotor, PidControl
*RightMotor)
{
ResetPidVariables(LeftMotor,RightMotor);
while(((right_encoder.Counts()) / 2.0) / ECPI < (TurnConstant/360*angle)){
LCD.Clear();
float LeftPower = LeftPidAdjustment(speed,LeftMotor);
float RightPower = RightPidAdjustment(speed,RightMotor);
left_motor.SetPercent(0 * LeftPower);
right_motor.SetPercent(1 *(RightPower));
LCD.WriteLine(left_encoder.Counts());
LCD.WriteLine(right_encoder.Counts());
Sleep(0.01);
}
//Turn off motors
right_motor.Stop();

left_motor.Stop();
}
void BackwardsDrive(float MaxSpeed, float distance, PidControl *LeftMotor,
PidControl *RightMotor)
{
ResetPidVariables(LeftMotor,RightMotor);
left_encoder.ResetCounts();
right_encoder.ResetCounts();
float CurrentSpeed = (1.0);
float RampRate = (1.1);
while(((left_encoder.Counts() + right_encoder.Counts()) / 2.0) / ECPI <
distance){
if (CurrentSpeed < MaxSpeed){
CurrentSpeed *= RampRate;
}
if (CurrentSpeed > MaxSpeed){
CurrentSpeed = MaxSpeed;
}
float LeftPower = LeftPidAdjustment(CurrentSpeed,LeftMotor);
float RightPower = RightPidAdjustment(CurrentSpeed,RightMotor);
left_motor.SetPercent(-1 * LeftPower);
right_motor.SetPercent(1 *RightPower);
Sleep(0.01);
}
//Turn off motors
right_motor.Stop();
left_motor.Stop();
}
void StartLight()
{
int start = 0;
while (start==0){
LCD.Clear();
if (CdsCell.Value() > CdsRedMax){
LCD.WriteLine("Waiting (Greater)");

LCD.WriteLine(CdsCell.Value());
}
if (CdsCell.Value() < CdsRedMin){
LCD.WriteLine("Waiting (Less)");
LCD.WriteLine(CdsCell.Value());
}
else if (CdsCell.Value() < CdsRedMax && CdsCell.Value()>CdsRedMin){
start = 1;
LCD.WriteLine("START");
}
Sleep (0.1);
}
}
void Initialize()
{
RCS.InitializeTouchMenu("0800AOJK"); // Run Menu to select Region (e.g., A,
B, C, D)
}
int readLightDisplay() {
float lightValue = CdsCell.Value();
LCD.Clear();
LCD.Write("Light Value: ");
LCD.WriteLine(lightValue);
if (lightValue >= CdsRedMin1 && lightValue <= CdsRedMax1) {
LCD.WriteLine("Light Color: RED");
lightValue = 1;
} else if (lightValue >= CdsBlueMin && lightValue <= CdsBlueMax) {
LCD.WriteLine("Light Color: BLUE");
lightValue = 2;
} else {
LCD.WriteLine("Light Color: blue");
lightValue = 2;
}
Sleep(0.25);
return lightValue;
}

void ReadSensors()
{
LCD.Clear();
LCD.WriteLine("Left:");
LCD.WriteLine(LeftOptosensor.Value());
LCD.WriteLine("Middle:");
LCD.WriteLine(MiddleOptosensor.Value());
LCD.WriteLine("Right:");
LCD.WriteLine(RightOptosensor.Value());
Sleep(0.25);
}
void StartButton(PidControl *LeftMotor, PidControl *RightMotor){
BackwardsDrive(12,0.7,LeftMotor,RightMotor);
}
void CompostBin (PidControl *LeftMotor, PidControl *RightMotor){
int Speed = 3.0;
// Allign with compost bin
Sleep(0.2);
ForwardDrive(6,23.75,LeftMotor,RightMotor);
LeftTurn(126,3,LeftMotor,RightMotor);
ForwardDrive(6.5,4.35,LeftMotor,RightMotor);
BigArm.SetDegree(165);
// Whack
Sleep(0.2);
BackwardsDrive(Speed,3.25,RightMotor,LeftMotor);
BigArm.SetDegree(30);
Sleep(0.2);
ForwardDrive(Speed,2.2,LeftMotor,RightMotor);
BigArm.SetDegree(165);
// Whack
Sleep(0.2);
BackwardsDrive(Speed,3,RightMotor,LeftMotor);
BigArm.SetDegree(30);
Sleep(0.2);
ForwardDrive(Speed,2.6,LeftMotor,RightMotor);
BigArm.SetDegree(150);

Sleep(1.0);
ForwardDrive(Speed,1.9,LeftMotor,RightMotor);
//Back Whack
Sleep(1.0);
BackwardsDrive(Speed,1,LeftMotor,RightMotor);
BigArm.SetDegree(30);
Sleep(0.5);
BackwardsDrive(Speed,3,RightMotor,LeftMotor);
BigArm.SetDegree(160);
Sleep(0.2);
ForwardDrive(Speed,2.2,LeftMotor,RightMotor);
BigArm.SetDegree(30);
Sleep(0.5);
BackwardsDrive(Speed,3,RightMotor,LeftMotor);
BigArm.SetDegree(160);
Sleep(0.2);
ForwardDrive(Speed,2.3,LeftMotor,RightMotor);
BigArm.SetDegree(30);
Sleep(1.0);
BigArm.SetDegree(160);
BigArm.Off();
Sleep(1.0);
// Allign with back wall
BackwardsDrive(6,15.5,LeftMotor,RightMotor);
BigArm.SetDegree(0);
Sleep(0.2);
ForwardDrive(6,14,LeftMotor,RightMotor);
LeftBackArcTurn(85.5,7,LeftMotor,RightMotor);
BackwardsDrive(6,4,LeftMotor,RightMotor);
}
void AppleBucket(PidControl *LeftMotor, PidControl *RightMotor){
// When alligned in fron of the bucket

BigArm.SetDegree(LowArmDegree);
Sleep(1.0);
ForwardDrive(6,4.5,LeftMotor,RightMotor);
BigArm.SetDegree(HighArmDegree-40);
Sleep(0.5);
BigArm.SetDegree(HighArmDegree-80);
Sleep(0.5);
BigArm.SetDegree(HighArmDegree-120);
Sleep(0.5);
// align with bottom of ramp
RightTurn(45,6,LeftMotor,RightMotor);
BackwardsDrive(7,6,LeftMotor,RightMotor);
LeftTurn(45,6,LeftMotor,RightMotor);
BackwardsDrive(7,22,LeftMotor,RightMotor);
ForwardDrive(4,2,LeftMotor,RightMotor);
RightArcTurn(87,6,LeftMotor,RightMotor);
ForwardDrive(10,37,LeftMotor,RightMotor);
Sleep(0.5);
BackwardsDrive(4,3,LeftMotor,RightMotor);
LeftTurn(3,20,LeftMotor,RightMotor);
BigArm.SetDegree(HighArmDegree);
Sleep(0.5);
BackwardsDrive(3.5,6.3,LeftMotor,RightMotor);
BigArm.SetDegree(0);
ForwardDrive(3.5,2,LeftMotor,RightMotor);
LeftTurn(95,4,LeftMotor,RightMotor);
BackwardsDrive(4,8,LeftMotor,RightMotor);
}
void Window(PidControl *LeftMotor, PidControl *RightMotor){
// When alligned with the wall by the window
ForwardDrive(5,14,LeftMotor,RightMotor);
LeftArcTurn(5,10,LeftMotor,RightMotor);

RightArcTurn(13,10,LeftMotor,RightMotor);
LeftArcTurn(5,10,LeftMotor,RightMotor);
RightArcTurn(13,10,LeftMotor,RightMotor);
LeftArcTurn(5,10,LeftMotor,RightMotor);
RightArcTurn(13,10,LeftMotor,RightMotor);
LeftArcTurn(5,10,LeftMotor,RightMotor);
RightArcTurn(13,10,LeftMotor,RightMotor);
LeftArcTurn(5,10,LeftMotor,RightMotor);
RightArcTurn(13,10,LeftMotor,RightMotor);
LeftArcTurn(5,10,LeftMotor,RightMotor);
Sleep(0.5);
// Backing up
/*
BackwardsDrive(6,4,LeftMotor,RightMotor);
RightTurn(16,3,LeftMotor,RightMotor);
ForwardDrive(4,4,LeftMotor,RightMotor);
LeftArcTurn(5,3,LeftMotor,RightMotor);
ForwardDrive(4,4,LeftMotor,RightMotor);
RightArcTurn(20,3,LeftMotor,RightMotor);
BackwardsDrive(6,3,LeftMotor,RightMotor);
LeftArcTurn(10,3,LeftMotor,RightMotor);
RightArcTurn(10,3,LeftMotor,RightMotor);
ForwardDrive(6,4,LeftMotor,RightMotor);
BackwardsDrive(20,10,LeftMotor,RightMotor);
Sleep(0.5);
RightBackArcTurn(14,10,LeftMotor,RightMotor);
LeftBackArcTurn(5,10,LeftMotor,RightMotor);
RightBackArcTurn(10,10,LeftMotor,RightMotor);
LeftBackArcTurn(5,10,LeftMotor,RightMotor);
RightBackArcTurn(10,10,LeftMotor,RightMotor);
LeftBackArcTurn(5,10,LeftMotor,RightMotor);
Sleep(100.0);

ForwardDrive(19,40,LeftMotor,RightMotor);
Sleep(0.5);
BackwardsDrive(6,3,LeftMotor,RightMotor);
Sleep(0.5);
RightArcTurn(20,6,LeftMotor,RightMotor);
RightArcTurn(25,6,LeftMotor,RightMotor);
ForwardDrive(15,12,LeftMotor,RightMotor);
Sleep(0.6);
BackwardsDrive(16,20,LeftMotor,RightMotor);
*/
BackwardsDrive(6,12,LeftMotor,RightMotor);
RightTurn(23,6,LeftMotor,RightMotor);
BackwardsDrive(6,20,LeftMotor,RightMotor);
}
void HumidifierButton(PidControl *LeftMotor, PidControl *RightMotor){
// Starting at the back wall alligned with the humidifier light
LeftArcTurn(3,6,LeftMotor,RightMotor);
ForwardDrive(6,18.2,LeftMotor,RightMotor);
int light = readLightDisplay();
BigArm.SetDegree(67);
Sleep(1.0);
if (light == 1){ //RED
RightTurn(45,4,LeftMotor,RightMotor);
ForwardDrive(4,3,LeftMotor,RightMotor);
LeftTurn(45,4,LeftMotor,RightMotor);
ForwardDrive(6,4,LeftMotor,RightMotor);
Sleep(0.5);
BackwardsDrive(4,2,LeftMotor,RightMotor);
LeftTurn(45,4,LeftMotor,RightMotor);
BackwardsDrive(4,3,LeftMotor,RightMotor);
RightTurn(135,4,LeftMotor,RightMotor);
BackwardsDrive(6,17,LeftMotor,RightMotor);

}
else if (light == 2){ //BLUE
LeftTurn(45,4,LeftMotor,RightMotor);
ForwardDrive(4,3,LeftMotor,RightMotor);
RightTurn(45,4,LeftMotor,RightMotor);
ForwardDrive(6,4,LeftMotor,RightMotor);
Sleep(0.5);
BackwardsDrive(4,2,LeftMotor,RightMotor);
LeftTurn(45,4,LeftMotor,RightMotor);
BackwardsDrive(4,3,LeftMotor,RightMotor);
RightTurn(135,4,LeftMotor,RightMotor);
BackwardsDrive(6,17,LeftMotor,RightMotor);
}
}
void Levers(PidControl *LeftMotor, PidControl *RightMotor){
int Lever = 0;
Lever = RCS.GetLever();
ForwardDrive(6,11,LeftMotor,RightMotor);
LeftTurn(90,6,LeftMotor,RightMotor);
if (Lever == 0){ //Left
BackwardsDrive(6,3,LeftMotor,RightMotor);
RightTurn(45,6,LeftMotor,RightMotor);
BigArm.SetDegree(170);
Sleep(1.5);
BigArm.SetDegree(0);
BackwardsDrive(4,6,LeftMotor,RightMotor);
BigArm.SetDegree(170);
ForwardDrive(4,7,LeftMotor,RightMotor);
Sleep(1.5);
BigArm.SetDegree(0);
Sleep(0.3);
BigArm.SetDegree(170);

}
else if (Lever == 1){ //Middle
BackwardsDrive(6,7.7,LeftMotor,RightMotor);
RightTurn(45,6,LeftMotor,RightMotor);
ForwardDrive(6,3,LeftMotor,RightMotor);
BigArm.SetDegree(170);
Sleep(1.5);
BackwardsDrive(4,8,LeftMotor,RightMotor);
BigArm.SetDegree(0);
BigArm.SetDegree(170);
ForwardDrive(4,8.5,LeftMotor,RightMotor);
Sleep(1.5);
BigArm.SetDegree(0);
Sleep(0.3);
BigArm.SetDegree(170);
}
else if (Lever == 2){ //Right
BackwardsDrive(6,14.2,LeftMotor,RightMotor);
RightTurn(45,6,LeftMotor,RightMotor);
ForwardDrive(6,8,LeftMotor,RightMotor);
BigArm.SetDegree(170);
Sleep(1.5);
BigArm.SetDegree(0);
BackwardsDrive(4,8,LeftMotor,RightMotor);
BigArm.SetDegree(170);
ForwardDrive(4,8,LeftMotor,RightMotor);
Sleep(1.5);
BigArm.SetDegree(0);
Sleep(0.3);
BigArm.SetDegree(170);
}
BackwardsDrive(6,5,LeftMotor,RightMotor);
BigArm.SetDegree(0);
BackwardsDrive(6,6,LeftMotor,RightMotor);
LeftTurn(45,6,LeftMotor,RightMotor);
BackwardsDrive(6,10,LeftMotor,RightMotor);

LeftArcTurn(90,6,LeftMotor,RightMotor);
ForwardDrive(12,40,LeftMotor,RightMotor);
LeftArcTurn(180,8,LeftMotor,RightMotor);
}
void FinishButton(PidControl *LeftMotor, PidControl *RightMotor){
}
int main()
{
PidControl LeftMotor;
PidControl RightMotor;
//Initialize Robot
BigArm.SetMin(900);
BigArm.SetMax(2500);
RCS.InitializeTouchMenu("0800A3OJK");
BigArm.SetDegree(0);
StartLight();
StartButton(&LeftMotor,&RightMotor);
CompostBin(&LeftMotor,&RightMotor);
AppleBucket(&LeftMotor,&RightMotor);
Window(&LeftMotor,&RightMotor);
HumidifierButton(&LeftMotor,&RightMotor);
Levers(&LeftMotor,&RightMotor);
Sleep(10.0);

ForwardDrive(6,11,&LeftMotor,&RightMotor);
RightTurn(45,4,&LeftMotor,&RightMotor);
ForwardDrive(5,9.0,&LeftMotor,&RightMotor);
BigArm.SetDegree(180);
Sleep(0.5);
BigArm.SetDegree(0);
BackwardsDrive(4,8,&LeftMotor,&RightMotor);
BigArm.SetDegree(180);
ForwardDrive(4,8,&LeftMotor,&RightMotor);
Sleep(1.5);
BigArm.SetDegree(0);
Sleep(0.3);
BigArm.SetDegree(180);
}
