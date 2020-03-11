#include "PID.h"

PID::PID(float Kc, float tauI, float tauD, float interval) {
 
    usingFeedForward = false;
    inAuto           = false;
 
    //Default the limits to the full range of I/O: 3.3V
    //Make sure to set these to more appropriate limits for
    //your application.
    setInputLimits(0.0, 3.3);
    setOutputLimits(0.0, 3.3);
 
    tSample_ = interval;
 
    setTunings(Kc, tauI, tauD);
 
    setPoint_             = 0.0;
    processVariable_      = 0.0;
    prevProcessVariable_  = 0.0;
    controllerOutput_     = 0.0;
    prevControllerOutput_ = 0.0;
 
    accError_ = 0.0;
    bias_     = 0.0;
    
    realOutput_ = 0.0;
 
}
 
void PID::setInputLimits(float inMin, float inMax) {
 
    //Make sure we haven't been given impossible values.
    if (inMin >= inMax) {
        return;
    }
 
    //Rescale the working variables to reflect the changes.
    prevProcessVariable_ *= (inMax - inMin) / inSpan_;
    accError_            *= (inMax - inMin) / inSpan_;
 
    //Make sure the working variables are within the new limits.
    if (prevProcessVariable_ > 1) {
        prevProcessVariable_ = 1;
    } else if (prevProcessVariable_ < 0) {
        prevProcessVariable_ = 0;
    }
 
    inMin_  = inMin;
    inMax_  = inMax;
    inSpan_ = inMax - inMin;
 
}
 
void PID::setOutputLimits(float outMin, float outMax) {
 
    //Make sure we haven't been given impossible values.
    if (outMin >= outMax) {
        return;
    }
 
    //Rescale the working variables to reflect the changes.
    prevControllerOutput_ *= (outMax - outMin) / outSpan_;
 
    //Make sure the working variables are within the new limits.
    if (prevControllerOutput_ > 1) {
        prevControllerOutput_ = 1;
    } else if (prevControllerOutput_ < 0) {
        prevControllerOutput_ = 0;
    }
 
    outMin_  = outMin;
    outMax_  = outMax;
    outSpan_ = outMax - outMin;
 
}
 
void PID::setTunings(float Kc, float tauI, float tauD) {
 
    //Verify that the tunings make sense.
    if (Kc == 0.0 || tauI < 0.0 || tauD < 0.0) {
        return;
    }
 
    //Store raw values to hand back to user on request.
    pParam_ = Kc;
    iParam_ = tauI;
    dParam_ = tauD;
 
    float tempTauR;
 
    if (tauI == 0.0) {
        tempTauR = 0.0;
    } else {
        tempTauR = (1.0 / tauI) * tSample_;
    }
 
    //For "bumpless transfer" we need to rescale the accumulated error.
    if (inAuto) {
        if (tempTauR == 0.0) {
            accError_ = 0.0;
        } else {
            accError_ *= (Kc_ * tauR_) / (Kc * tempTauR);
        }
    }
 
    Kc_   = Kc;
    tauR_ = tempTauR;
    tauD_ = tauD / tSample_;
 
}
 
void PID::reset(void) {
 
    float scaledBias = 0.0;
 
    if (usingFeedForward) {
        scaledBias = (bias_ - outMin_) / outSpan_;
    } else {
        scaledBias = (realOutput_ - outMin_) / outSpan_;
    }
 
    prevControllerOutput_ = scaledBias;
    prevProcessVariable_  = (processVariable_ - inMin_) / inSpan_;
 
    //Clear any error in the integral.
    accError_ = 0;
 
}
 
void PID::setMode(int mode) {
 
    //We were in manual, and we just got set to auto.
    //Reset the controller internals.
    if (mode != 0 && !inAuto) {
        reset();
    }
 
    inAuto = (mode != 0);
 
}
 
void PID::setInterval(float interval) {
 
    if (interval > 0) {
        //Convert the time-based tunings to reflect this change.
        tauR_     *= (interval / tSample_);
        accError_ *= (tSample_ / interval);
        tauD_     *= (interval / tSample_);
        tSample_   = interval;
    }
 
}
 
void PID::setSetPoint(float sp) {
 
    setPoint_ = sp;
 
}
 
void PID::setProcessValue(float pv) {
 
    processVariable_ = pv;
 
}
 
void PID::setBias(float bias){
 
    bias_ = bias;
    usingFeedForward = 1;
 
}

float map2(float in, float inMin, float inMax, float outMin,
          float outMax) { // Function to scale the inputs to desired outputs
  // check it's within the range
  if (inMin < inMax) {
    if (in <= inMin)
      return outMin;
    if (in >= inMax)
      return outMax;
  } else { // cope with input range being backwards.
    if (in >= inMin)
      return outMin;
    if (in <= inMax)
      return outMax;
  }
  // calculate how far into the range we are
  float scale = (in - inMin) / (inMax - inMin);
  // calculate the output.
  return outMin + scale * (outMax - outMin);
}

float PID::compute() {
 
    //Pull in the input and setpoint, and scale them into percent span.
    float scaledPV = (processVariable_ - inMin_) / inSpan_;
 
    if (scaledPV > 1.0) {
        scaledPV = 1.0;
    } else if (scaledPV < 0.0) {
        scaledPV = 0.0;
    }
 
    float scaledSP = (setPoint_ - inMin_) / inSpan_;
    if (scaledSP > 1.0) {
        scaledSP = 1;
    } else if (scaledSP < 0.0) {
        scaledSP = 0;
    }
 
    float error = scaledSP - scaledPV;

    //Check and see if the output is pegged at a limit and only
    //integrate if it is not. This is to prevent reset-windup.
    if (!(prevControllerOutput_ >= 1 && error > 0) && !(prevControllerOutput_ <= -1.0 && error < 0)) {
        accError_ += error;
    }
 
    //Compute the current slope of the input signal.
    float dMeas = (scaledPV - prevProcessVariable_) / tSample_;
 
    float scaledBias = 0.0;
 
    if (usingFeedForward) {
        scaledBias = (bias_ - outMin_) / outSpan_;
    }
 
    //Perform the PID calculation.
    controllerOutput_ = scaledBias + Kc_ * (error + (tauR_ * accError_) - (tauD_ * dMeas));
 
    //Make sure the computed output is within output constraints.
    if (controllerOutput_ < -1.0) {
        controllerOutput_ = -1.0;
    } else if (controllerOutput_ > 1.0) {
        controllerOutput_ = 1.0;
    }
 
    //Remember this output for the windup check next time.
    prevControllerOutput_ = controllerOutput_;
    //Remember the input for the derivative calculation next time.
    prevProcessVariable_  = scaledPV;
//  float output=(controllerOutput_ * outSpan_) - outMin_;
float output=map2(controllerOutput_,-1.0,1.0,outMin_,outMax_);
    //Scale the output from percent span back out to a real world number.
    //  printf(" scaledPV: %f scaledSP: %f error:%f accError: %f controller output: %f output: %f\n\r",scaledPV,scaledSP,error,accError_,controllerOutput_, output);
    return (output);
 
}
 
float PID::getInMin() {
 
    return inMin_;
 
}
 
float PID::getInMax() {
 
    return inMax_;
 
}
 
float PID::getOutMin() {
 
    return outMin_;
 
}
 
float PID::getOutMax() {
 
    return outMax_;
 
}
 
float PID::getInterval() {
 
    return tSample_;
 
}
 
float PID::getPParam() {
 
    return pParam_;
 
}
 
float PID::getIParam() {
 
    return iParam_;
 
}
 
float PID::getDParam() {
 
    return dParam_;
 
}
