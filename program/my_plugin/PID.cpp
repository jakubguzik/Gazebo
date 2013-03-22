#include <iostream>
#include <sys/time.h>
#include "PID.hh"
using namespace std;

PID::PID(double P, double I, double D): PP(P), II(I), DD(D){
  if (gettimeofday(&TPoprz, NULL) == -1) {
   cerr<<"Failed to get start time"<<endl;
  }
}


double PID::GetTimeStep(){
  double tdif;
  struct timeval TObecny;

  if (gettimeofday(&TObecny, NULL) == -1) {
    cerr<<"Failed to get start time"<<endl;
    return 1;
  }

  tdif = MILLION*(TObecny.tv_sec - TPoprz.tv_sec) +
    TObecny.tv_usec - TPoprz.tv_usec;

  TPoprz=TObecny;

  return tdif;
}

void PID::SetP(double P){
  if(P>=0) PP=P;
  else cerr<<"P<0"<<endl;
} 
void PID::SetI(double I){
  if(I>=0) II=I;
  else cerr<<"I<0"<<endl;
}
void PID::SetD(double D){
  if(D>=0) DD=D;
  else cerr<<"D<0"<<endl;
} 
void PID::SetSP(double SP){
  WZadana=SP;
}

double PID::Control(double PV){
  double err=WZadana-PV;
  double result=0;
  double timestep=GetTimeStep();
  result+=err*PP;
  result+=DD*(Poprzedni-PV)/timestep;
  result+=Calka+err*II*timestep;
  return result;
}
