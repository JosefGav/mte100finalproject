// ---------------------- includes & config ----------------------
#include "vex.h"
using namespace vex;

brain Brain;
inertial BrainInertial = inertial();
motor MotorLeft      = motor(PORT3,  false);
motor MotorRight     = motor(PORT6,  true);
motor MotorDispense  = motor(PORT1,  false);  // <- change if needed

// ---------------------- misc init ----------------------
void initializeRandomSeed(){
  wait(100,msec);
  double xAxis = BrainInertial.acceleration(xaxis) * 1000;
  double yAxis = BrainInertial.acceleration(yaxis) * 1000;
  double zAxis = BrainInertial.acceleration(zaxis) * 1000;
  int seed = int(xAxis + yAxis + zAxis);
  srand(seed);
}

void vexcodeInit() {
  initializeRandomSeed();
}

// ---------------------- setup ----------------------
void configureAllSensors(){
  BrainInertial.calibrate();
  wait(2, seconds);
  BrainInertial.setHeading(0, degrees);
  BrainInertial.setRotation(0, degrees);
  MotorLeft.setPosition(0, turns);
  MotorRight.setPosition(0, turns);
  MotorDispense.setPosition(0, deg);
}

// ---------------------- helpers ----------------------
static inline double clampAbs(double v, double mn, double mx){
  double a = fabs(v);
  if (a < mn) a = mn;
  if (a > mx) a = mx;
  return copysign(a, v);
}

static inline double normDeg(double a){
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

// ---------------------- PID rotate to absolute heading ----------------------
bool rotateToHeadingPID(double targetDeg,
                        double kp=1.2, double ki=0.0, double kd=0.12,
                        int timeoutMs=2000, double tolDeg=2.0)
{
  const double maxPct = 70.0;
  const double minPct = 8.0;
  const double iLimit = 25.0;   // integral guard (deg)
  const int    loopDt = 15;     // ms

  // set once; keep while-loop empty of spin() calls
  MotorLeft.setStopping(brake);
  MotorRight.setStopping(brake);
  MotorLeft.spin(forward);
  MotorRight.spin(forward);

  timer t;
  double err      = normDeg(targetDeg - BrainInertial.heading(degrees));
  double prevErr  = err;
  double integral = 0.0;

  while (fabs(err) > tolDeg && t.time(msec) < timeoutMs) {
    if (fabs(err) < iLimit) integral += err * (loopDt/1000.0); else integral = 0.0;
    double derivative = (err - prevErr) / (loopDt/1000.0);

    // control effort (u>0 => need CW; u<0 => need CCW)
    double u = kp*err + ki*integral + kd*derivative;

    // convert to tank turn velocities: left = +u, right = -u
    double leftPct  = clampAbs( u, minPct, maxPct);
    double rightPct = clampAbs(-u, minPct, maxPct);

    MotorLeft.setVelocity(leftPct,  percent);
    MotorRight.setVelocity(rightPct, percent);

    wait(loopDt, msec);
    prevErr = err;
    err = normDeg(targetDeg - BrainInertial.heading(degrees));
  }

  MotorLeft.stop();
  MotorRight.stop();
  return fabs(err) <= tolDeg;
}

// ---------------------- dispense one card (v3: anti-double-deal) ----------------------
const double DEG_PER_CARD = 200.0;      // tune for your feeder
const int    MAX_MS = 240;              // watchdog for dispense motion

void dispenseOneCard(){
  // ============================================================
  // Dispense forward as usual
  // ============================================================
  double startDispense = MotorDispense.position(deg);
  MotorDispense.setVelocity(90, percent);
  MotorDispense.spin(forward);

  timer t;
  while ((MotorDispense.position(deg) - startDispense) < DEG_PER_CARD
         && t.time(msec) < MAX_MS) { }

  MotorDispense.stop(brake);

  // ============================================================
  // NEW: After dispensing, run motor BACKWARDS for the same time
  //      to retract any protruding cards (anti-double-deal)
  // ============================================================
  int dispenseTime = t.time(msec);  // get actual dispense time

  MotorDispense.setVelocity(90, percent);  // same speed as forward
  MotorDispense.spin(reverse);             // spin backwards

  wait(dispenseTime, msec);  // run backwards for same duration

  MotorDispense.stop(brake);
}

// ---------------------- main: deal to 2 players forever ----------------------
int main() {
  vexcodeInit();
  configureAllSensors();

  const double seats[2] = { 0.0, 180.0};

  while (true) {
    for (int i = 0; i < 2; ++i) {
      rotateToHeadingPID(seats[i], 1.25, 0.0, 0.12, 2000, 2.0);
      dispenseOneCard();
      wait(80, msec);
    }
  }
}
