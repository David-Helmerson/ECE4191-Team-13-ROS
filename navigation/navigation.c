#include math.h
#include stdio.h

int main()
{

// float r,d，v,w;
// int ticks_per_rev, ticks_l, ticks_r;
// float S_l = 2*pi()*r*ticks_l/ticks_per_rev;
// float S_r = 2*pi()*r*ticks_r/ticks_per_rev;

// float fai = (S_r - S_l)/d;

float d; //distance between two wheels
float t; // time to reach the destination
float r_wheel; // radien of wheels
float x,y;   // destination
float a[2] = [0 0],b[2] = [x y]; //oringinal opint and destination point
float theta = atan(y/x);
float r = (pow(x,2)+pow(y,2))/(2*sqrt((pow(x,2)+pow(y,2)))*cos(theta)); //radien of mid point
float r_right = r - d/2;
float r_left = r + d/2;
float v_right = theta*2*pi()*r_right/(360*theta); // linear speed of right wheel
float v_left = theta*2*pi()*r_left/(360*theta); // linear speed of left wheel
float w_r = v_right/r_wheel; // angular speed of right wheel
float w_l = v_left/r_wheel; // angular speed of left wheel

}