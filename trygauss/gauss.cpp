#include <iostream>
#include <cmath>
#include <fstream>
using namespace std;

#define SQUARE(x) ((x)*(x))
double u = 47;
double sigma = 6;
const double pi = 3.141592654;

double gauss(double);
int main(){
	ofstream g("gauss.log");
	double y;
	for(double x=0;x<=63;x+=0.5){
		y = gauss(x);
		cout<<x<<" "<<y<<endl;
		g<<x<<" "<<y<<endl;
	}
	return 0;
}

double gauss(double x){
	double b = SQUARE(x-u)/(2*SQUARE(sigma));
	return exp(b*-1.);
}
