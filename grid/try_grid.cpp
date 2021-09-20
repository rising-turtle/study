#include <iostream>
#include <cmath>

using namespace std; 

double mod(double value, double modulus) {
    return fmod(fmod(value, modulus) + modulus, modulus);
}


double intbound(double s, double ds) {
    // Find the smallest positive t such that s+t*ds is an integer.
    if (ds < 0) {
	return intbound(-s, -ds);
    } else {
	// cout<<"s: "<<s<<" ds: "<<ds<<endl; 
	s = mod(s, 1);
	// cout<<"s: "<<s<<" ds: "<<ds<<endl; 
	// problem is now s+t*ds = 1
	return (1 - s) / ds;
    }
}

int signum(int x) {
    return x == 0 ? 0 : x < 0 ? -1 : 1;
    }


int main(int argc, char* argv[])
{
    
    double SX = 1.1; 
    double SY = 1.5; 
    double EX = 5.2;
    double EY = 3.3; 
    
    int x = std::floor(SX); 
    int y = floor(SY); 
    int endX = floor(EX); 
    int endY = floor(EY); 
    

    int dx = endX - x; 
    int dy = endY - y; 
    
    int stepX = signum(dx); 
    int stepY = signum(dy); 

    double tMaxX = intbound(SX, dx); 
    double tMaxY = intbound(SY, dy); 

    double tDeltaX = 1./dx; 
    double tDeltaY = 1./dy; 
    
    while(1){

	if(x > endX || y > endY)
	    break; 
	
	cout <<"current grid: ( "<<x<<", "<<y<<") tMaxX: "<<tMaxX<<" tMaxY: "<<tMaxY<<endl;  


	if(tMaxX < tMaxY){
	    tMaxX += tDeltaX; 
	    x += stepX; 
	}else{
	    tMaxY += tDeltaY; 
	    y += stepY; 
	}
    
    }
    
    return 0; 

}










