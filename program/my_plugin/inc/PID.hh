#ifndef PID_HH
#define PID_HH

#define MILLION 1e6L

class PID
{
  struct timeval TPoprz;

  double Calka;
  double Poprzedni;
  double PP, II, DD;
  double kat;
  double Sterowanie;
  double WZadana;

  double GetTimeStep();

public:
  PID(double P, double I, double D);

  void SetP(double P);
  void SetI(double I);
  void SetD(double D);
  void SetSP(double SP);

  double Control(double PV);
};

#endif				// PID_HH
