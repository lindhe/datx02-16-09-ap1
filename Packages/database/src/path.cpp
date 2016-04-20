#include <iostream>
#include <fstream>
#include <getopt.h>
#include <math.h>
using namespace std;
#define limit 2*3.1415
int main (int argc, char** argv) {
  const char* options = "ho:x:y:s:l";
  int c;
  int step = 100;
  int x_radius = 0;
  int y_radius = 0;
  int offset = 0;
  bool line = false;
  while ((c = getopt(argc, argv, options)) != -1) {
      switch (c) {
          case 'o': offset = atoi(optarg); break;
          case 'x': x_radius = atoi(optarg); break;
          case 'y': y_radius = atoi(optarg); break;
          case 's': step = atoi(optarg); break;
          case 'l': line = true; break;
          case 'h':
            cout << "-o LENGTH      Offset in millimeter" << endl;
            cout << "-x LENGTH      Radius x-axis in millimeter" << endl;
            cout << "-y LENGTH      Radius y-axis in millimeter" << endl;
            cout << "-s NUMBER      Number of points on the track" << endl;
            cout << "-l def:-2000   Line from -radius x-axis to +radius x-axis" << endl;
            break;
      }
  }
   ofstream file;
   file.open ("data.txt");
   if(!line){
   
       for(double i=0; i < limit; i += limit/step) {
           file << (int)(x_radius*cos(i)+offset) << " " << (int)(y_radius*sin(i)+offset) << endl;
       }
   }else{

       for(double i=-x_radius; i < x_radius; i += 2*x_radius/step) {
           file << i << " " << -2000 << endl;
       }
   } 
   file.close();
   return 0;
}
