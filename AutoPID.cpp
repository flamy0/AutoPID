#include "AutoPID.h"

AutoPID::AutoPID(double *input, double *setpoint, double *output, double outputMin, double outputMax,
                 double Kp, double Ki, double Kd) {
  _input = input;
  _setpoint = setpoint;
  _output = output;
  _outputMin = outputMin;
  _outputMax = outputMax;
  setGains(Kp, Ki, Kd);
  _timeStep = 1000;
}//AutoPID::AutoPID

void AutoPID::setGains(double Kp, double Ki, double Kd) {
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
}//AutoPID::setControllerParams

void AutoPID::setBangBang(double bangOn, double bangOff) {
  _bangOn = bangOn;
  _bangOff = bangOff;
}//void AutoPID::setBangBang

void AutoPID::setBangBang(double bangRange) {
  setBangBang(bangRange, bangRange);
}//void AutoPID::setBangBang

void AutoPID::setOutputRange(double outputMin, double outputMax) {
  _outputMin = outputMin;
  _outputMax = outputMax;
}//void AutoPID::setOutputRange

void AutoPID::setTimeStep(unsigned long timeStep){
  _timeStep = timeStep;
}

bool AutoPID::atSetPoint(double threshold) {
  return abs(*_setpoint - *_input) <= threshold;
}//bool AutoPID::atSetPoint

void AutoPID::run() {
  if (_stopped) {
    _stopped = false;
    reset();
  }
  
  if(!_paused) {
  
  //if bang thresholds are defined and we're outside of them, use bang-bang control
  if (_bangOn && ((*_setpoint - *_input) > _bangOn)) {
    *_output = _outputMax;
    _lastStep = millis();
  } else if (_bangOff && ((*_input - *_setpoint) > _bangOff)) {
    *_output = _outputMin;
    _lastStep = millis();
  } else {                                    //otherwise use PID control
    unsigned long _dT = millis() - _lastStep;   //calculate time since last update
    if (_dT >= _timeStep) {                     //if long enough, do PID calculations
      _lastStep = millis();
	  _ran = true;
      double _error = *_setpoint - *_input;
	  //Serial.print("Error: ");
	  //Serial.println(_error);
      _integral += (_error + _previousError) / 2 * _dT / 1000.0;   //Riemann sum integral
	  //_integral = constrain(_integral, _outputMin/_Ki, _outputMax/_Ki);
      //Serial.print("Integral: ");
	  //Serial.println(_integral);
	  double _dError = (_error - _previousError) / _dT / 1000.0;   //derivative
	  //Serial.print("dError: ");
	  //Serial.println(_dError);
      _previousError = _error;
      double PID = (_Kp * _error) + (_Ki * _integral) + (_Kd * _dError);
	  Serial.print(*_input);
	  Serial.print(",");
	  Serial.print(_Kp);
	  Serial.print(",");
	  Serial.print(_Ki);
	  Serial.print(",");
	  Serial.print(_Kd);
	  //Serial.print(",");
	  //Serial.print(_error);Serial.print(",");Serial.print(_integral);Serial.print(",");Serial.print(_dError);
	  Serial.print(",");
      Serial.println(PID);
	  //*_output = _outputMin + (constrain(PID, 0, 1) * (_outputMax - _outputMin));
      *_output = constrain(PID, _outputMin, _outputMax);
	  //*_output = 50;
    }
  }
  
  }
}//void AutoPID::run

void AutoPID::pause() {
  _paused = true;
}

void AutoPID::unpause() {
  _lastStep = millis();
  _paused = false;
}

void AutoPID::stop() {
  _stopped = true;
  reset();
}
void AutoPID::reset() {
  _lastStep = millis();
  _integral = 0;
  _previousError = 0;
}

bool AutoPID::isPaused(){
  return _paused;
}

bool AutoPID::isStopped(){
  return _stopped;
}

bool AutoPID::hasRun(){
  if(_ran == true) {
    _ran = false;
    return true;
  }
  return false;
}

double AutoPID::getIntegral(){
  return _integral;
}

void AutoPID::setIntegral(double integral){
  _integral = integral;
}

void AutoPIDRelay::run() {
  AutoPID::run();
  while ((millis() - _lastPulseTime) > _pulseWidth) _lastPulseTime += _pulseWidth;
  *_relayState = ((millis() - _lastPulseTime) < (_pulseValue * _pulseWidth));
}


double AutoPIDRelay::getPulseValue(){
  return (isStopped()?0:_pulseValue);
}


