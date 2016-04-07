#include <iostream>
#include <fstream>
#include <math.h>
using namespace std;
#define step 50
#define radius 4000
#define offset 1000
int main () {
   ofstream file;
   file.open ("data.txt");
   for(int i= radius; i >= -radius; i = i - step){
        file << i+offset << " " << (int)sqrt(pow(radius,2) - pow(i, 2))+offset << endl;
   } 
   for(int i= -radius+step; i < radius; i = i + step){
        file << i+offset << " " << (int)-sqrt(pow(radius,2) - pow(i, 2))+offset << endl;
   }
   file.close();
   return 0;
}
