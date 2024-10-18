#include <main.h>
#include <cmath>

const long double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067982148086513282306647093844L;

// define sensors
int gpsPortLocal = 12;
int distancePortLocal = 2;

pros::Gps gps(gpsPortLocal, 0.0375, 0.15);
pros::Distance distanceSensor(distancePortLocal);

// define motor groups
struct {int8_t leftPortA = -1; int8_t leftPortB = -11;} drivetrainLeftPortLocal;
struct {int8_t rightPortA = 10; int8_t rightPortB = 20;} drivetrainRightPortLocal;

pros::MotorGroup drivetrainLeft({drivetrainLeftPortLocal.leftPortA, drivetrainLeftPortLocal.leftPortB}, pros::v5::MotorGears::green);
pros::MotorGroup drivetrainRight({drivetrainRightPortLocal.rightPortA, drivetrainRightPortLocal.rightPortB}, pros::v5::MotorGears::green);

double bearingToPoint(double finalX, double finalY) {
    // declare bearing to avoid issues
    double bearing;

    // get integer values for all coordinates
    pros::gps_position_s_t
    position = gps.get_position();

    double startX = position.x;
    double startY = position.y;

    // if-else to avoid an undefined error
    if ((finalX - startX) == 0) {
        bearing = 0;
    }

    else {
        // get slope
        double m = (finalY - startY) / (finalX - startX);

        // get bearing from x-axis in radians
        double bearingInRadians = atan(m);

        // convert bearing into degrees
        double bearingInDegrees = bearingInRadians * (180 / pi);

        // convert from x-axis to y-axis
        bearing = -1 * (bearingInDegrees - 90);
    }

    if (startX > finalX) {
        bearing = bearing + 180;
    }

    else if (startX == finalX && startY > finalY) {
        bearing = bearing + 180;
    }

    return bearing;
}

double distanceToPoint(double finalX, double finalY) {
    // get integer values for all coordinates
    pros::gps_position_s_t position;
    position = gps.get_position();

    double startX = round(position.x);
    double startY = round(position.y);

    // get the distance over the x/y axes
    double legX = abs(finalX - startX); 
    double legY = abs(finalY - startY);

    // square legs and add for Pythagorean Thereom
    double distanceSquare = pow(legX, 2) + pow(legY, 2);

    // get sqaure root
    double distance = sqrt(distanceSquare);

    return distance;
}

void pointTo(double finalX, double finalY) {
    // get heading to point
    double finalHeading = bearingToPoint(finalX, finalY);

    // get current heading
    double currentHeading = gps.get_heading();

    if (currentHeading > 90) {
        currentHeading = currentHeading - 90;
    }

    else {
        currentHeading = currentHeading + 270;
    }

    // get which angle of turning is greater
    double clockWise = (finalHeading + 360) - currentHeading;

    if (clockWise < 360 && clockWise > 180) {
        while (((finalHeading + 1) < currentHeading) && ((finalHeading - 1) > currentHeading)) {
            drivetrainLeft.move(32);
            drivetrainRight.move(-32);

            currentHeading = gps.get_heading();
        }
    }

    else {
        while (((finalHeading + 1) < currentHeading) && ((finalHeading - 1) > currentHeading)) {
            drivetrainLeft.move(-32);
            drivetrainRight.move(32);

            currentHeading = gps.get_heading();
        }
    }
}

void goTo(double finalX, double finalY) {
    // get distance to point
    double distancePoint = distanceToPoint(finalX, finalY);

    // rotate to new heading
    pointTo(finalX, finalY);

    // get distance to wall
    double distanceToObject = distanceSensor.get_distance();

    // convert wall distance to meters
    distanceToObject = distanceToObject / 1000;

    // get distance to wall at final point
    double distanceToObjectFinal = distanceToObject - distancePoint;

    // drive to point
    while (distanceToObject > distanceToObjectFinal) {
        drivetrainLeft.move(32);
        drivetrainRight.move(32);

        distanceToObject = distanceSensor.get_distance();
    }

    // stop
    drivetrainLeft.brake();
    drivetrainRight.brake();
}