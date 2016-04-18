#include <iostream>
#include <fstream>
#include <math.h>
using namespace std;
#define step 100
#define radius 3500
#define offset 500
#define limit 2*3.1415
int main () {
   ofstream file;
   file.open ("data.txt");
/*   for(int i= radius; i >= -radius; i = i - step){
        file << i+offset << " " << (int)sqrt(pow(radius,2) - pow(i, 2))+offset << endl;
   } 
   for(int i= -radius+step; i < radius; i = i + step){
        file << i+offset << " " << (int)-sqrt(pow(radius,2) - pow(i, 2))+offset << endl;
   }*/
   for(double i=0; i < limit; i += limit/step) {
        cout << i << " " << limit << endl;
        file << (int)(radius*cos(i)+offset) << " " << (int)(radius*sin(i)+offset) << endl;
   }
   file.close();
   return 0;
}
