#include <Servo.h>
#include "BraccioRobot.h"
#include <math.h>



// Link parameters (mm)
const double l0 = 71.5;  // base-to-shoulder height
const double L1 = 125.0; // upper arm length
const double L2 = 125.0; // forearm length
const double L3 = 192.0; // tool length (wrist-to-gripper tip)

// Desired end-effector pitch (radians): vertical down = -90
const double phi = -PI / 2;
Position armPosition;

void setup() {
    Serial.begin(115200);
    BraccioRobot.init();
    Serial.println("Braccio Pure IK Ready");
}

/**
 * Read pick and place coordinates: G:x1,y1,z1,x2,y2,z2
 */
bool readCoordinates(double &xP, double &yP, double &zP,
                     double &xL, double &yL, double &zL) {
    if (!Serial.available()) return false;

    String line = Serial.readStringUntil('\n');
    if (line.length() < 2 || line.charAt(0) != 'G') return false;

    line = line.substring(2);
    double vals[6];
    int idx = 0;
    char buf[64];
    
    line.toCharArray(buf, sizeof(buf));
    char *tok = strtok(buf, ",");
    
    while (tok && idx < 6) {
        vals[idx++] = atof(tok);
        tok = strtok(NULL, ",");
    }

    if (idx != 6) return false;

    xP = vals[0]; yP = vals[1]; zP = vals[2];
    xL = vals[3]; yL = vals[4]; zL = vals[5];
    return true;
}

/**
 * Compute IK (radians) then convert to degrees
 */
void computeIK(double x, double y, double z,
               double &t1, double &t2, double &t3, double &t4) {
    
    t1 = atan2(y, x);
    double r = sqrt(x * x + y * y);
    double s = z - l0;
    double D = sqrt(r * r + s * s);
    double Dp = D - L3;

    // Law of Cosines for t3
    double cosA = (L1 * L1 + L2 * L2 - Dp * Dp) / (2 * L1 * L2);
    cosA = constrain(cosA, -1, 1);
    double alpha = acos(cosA);
    t3 = M_PI - alpha;

    // Law of Cosines for t2
    double beta = atan2(s, r);
    double cosG = (L1 * L1 + Dp * Dp - L2 * L2) / (2 * L1 * Dp);
    cosG = constrain(cosG, -1, 1);
    double gamma = acos(cosG);
    t2 = beta + gamma;

    // Solve for wrist pitch
    t4 = phi - (t2 + t3);

    // Convert to degrees
    t1 *= 180.0 / PI;
    t2 *= 180.0 / PI;
    t3 *= 180.0 / PI;
    t4 *= 180.0 / PI;
}

void moveToXYZ(double x, double y, double z, bool closeGripper) {
    double t1, t2, t3, t4;
    computeIK(x, y, z, t1, t2, t3, t4);

    // Map IK angles to Braccio servo ranges
    int b  = round(180 - t1),
        s  = round(t2),
        e  = round(t3),
        w  = round(180 - t4),
        wr = 90,
        g  = closeGripper ? 73 : 10; // Updated 0 to 10 for safer motor range

    armPosition.set(b, s, e, w, wr, g);
    BraccioRobot.moveToPosition(armPosition, 100);
    delay(300);
}

void pickAndPlace(double xP, double yP, double zP,
                  double xL, double yL, double zL) {
    
    moveToXYZ(xP, yP, zP + 100, false); // Approach above pick
    moveToXYZ(xP, yP, zP, false);       // Lower to pick
    moveToXYZ(xP, yP, zP, true);        // Close gripper
    moveToXYZ(xP, yP, zP + 100, true);  // Lift

    moveToXYZ(xL, yL, zL + 100, true);  // Approach above place
    moveToXYZ(xL, yL, zL, true);        // Lower to place
    moveToXYZ(xL, yL, zL, false);       // Open gripper
    moveToXYZ(xL, yL, zL + 100, false); // Clear

    // Return to home position
    armPosition.set(90, 90, 90, 90, 90, 73);
    BraccioRobot.moveToPosition(armPosition, 150);
    delay(300);
}

void loop() {
    double xP, yP, zP, xL, yL, zL;
    if (readCoordinates(xP, yP, zP, xL, yL, zL)) {
        pickAndPlace(xP, yP, zP, xL, yL, zL);
        Serial.println("Done");
    }
}