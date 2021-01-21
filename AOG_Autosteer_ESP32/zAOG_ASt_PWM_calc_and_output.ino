void calcSteeringPID(void)
{
    if (steerSet.aogVersion == 0) {
        //Proportional
        pValue = steerSet.Kp * steerAngleError * steerSet.Ko;

        /* //Derivative
         dError = steerAngleError - lastLastError;
         dValue = steerSet.Kd * (dError) * steerSet.Ko;

         //save history of errors
         lastLastError = lastError;
         lastError = steerAngleError;
        */
        /*Serial.print(steerSet.Kp); Serial.print(":Kp  Ko: "); Serial.print(steerSet.Ko);
        Serial.print(" steerAngleError: "); Serial.println(steerAngleError);
        Serial.print(" min PWM: "); Serial.print(steerSet.minPWMValue);
       */
        drive = pValue;// + dValue;
        pwmDrive = (constrain(drive, -255, 255));

        //add throttle factor so no delay from motor resistance.
        if (pwmDrive < 0) pwmDrive -= steerSet.minPWMValue;
        else if (pwmDrive > 0) pwmDrive += steerSet.minPWMValue;

        if (pwmDrive > 255) pwmDrive = 255;
        if (pwmDrive < -255) pwmDrive = -255;
        //Serial.print(" PWM value: "); Serial.println(pwmDrive);
    }
    else {
        //Proportional only
        pValue = steerSet.Kp * steerAngleError;
        pwmDrive = (int)pValue;

        errorAbs = abs(steerAngleError);
        float newMax = 0;

        if (errorAbs < steerSet.MotorSlowDriveDegrees)
        {
            newMax = (errorAbs * highLowPerDeg) + steerSet.deadZone;
        }
        else newMax = steerSet.maxPWM;

        //add min throttle factor so no delay from motor resistance.
        if (pwmDrive < 0) pwmDrive -= steerSet.minPWM;
        else if (pwmDrive > 0) pwmDrive += steerSet.minPWM;

        //Serial.print(newMax); //The actual steering angle in degrees
        //Serial.print(",");

     //limit the pwm drive
        if (pwmDrive > newMax) pwmDrive = newMax;
        if (pwmDrive < -newMax) pwmDrive = -newMax;

        if (steerSet.MotorDriveDirection) pwmDrive *= -1;
        if (steerSet.debugmode) { Serial.print("PWM for output: ");  Serial.println(pwmDrive); }
    }
}

//-------------------------------------------------------------------------------------------------
// select output Driver
//---------------------------------------------------------------------
void motorDrive(void) 
{
  switch (steerSet.output_type) {
    case 1:
      motorDrive_Cytron();
      break;
    case 2:
      motorDrive_IBT_Mot();
      break;
    case 3:
      motorDrive_IBT_PWM();
      break;
    case 4:
      motorDrive_IBT_Danfoss();
      break;
    case 5:
      motorDrive_Keya_v2020();
      break;
    default:
      // if nothing else matches no Output
    break;
  }
}
 
//---------------------------------------------------------------------
// Used with Cytron MD30C Driver
// Steering Motor
// Dir + PWM Signal
//---------------------------------------------------------------------
void motorDrive_Cytron(void) 
  {    
    pwmDisplay = pwmDrive;
    if (pwmDrive >= 0) ledcWrite(1, 255);  // channel 1 = DIR_PIN  //set the correct direction
    else   
    {
      ledcWrite(1, 0);  // channel 1 = DIR_PIN 
      pwmDrive = -1 * pwmDrive;  
   }
    ledcWrite(0, pwmDrive);  // channel 0 = PWM_PIN
  }


//---------------------------------------------------------------------
// Used with IBT 2  Driver
// Steering Motor
// PWM Left + PWM Right Signal
// Same Code as the PWM Valve
//---------------------------------------------------------------------
void motorDrive_IBT_Mot(void) 
  {   
   if (steerEnable == false) pwmDrive=0;
   
    pwmDisplay = pwmDrive; 
  
  if (pwmDrive > 0)
    {
      ledcWrite(0, pwmDrive);  // channel 0 = PWM_PIN
      ledcWrite(1, 0);
      //digitalWrite(DIR_PIN, LOW);
    }
    
  if (pwmDrive <= 0)
    {
      pwmDrive = -1 * pwmDrive;  
      ledcWrite(1, pwmDrive);  // channel 1 = DIR_PIN
      ledcWrite(0, 0);
      //digitalWrite(PWM_PIN, LOW);
    } 
  }

//---------------------------------------------------------------------
// Used with 2-Coil PWM Valves 
// No coil powered = Center
// Coil 1 steer right connected to PWM_PIN 11
// Coil 2 steer left  connected to DIR_PIN 10
//---------------------------------------------------------------------
void motorDrive_IBT_PWM(void)
 { 
  if (steerEnable == false) pwmDrive=0;
   
  pwmDisplay = pwmDrive; 
  
  if (pwmDrive > 0)
    {
      ledcWrite(0, pwmDrive);  // channel 0 = PWM_PIN
      ledcWrite(1, 0);
      //digitalWrite(DIR_PIN, LOW);
    }
    
  if (pwmDrive <= 0)
    {
      pwmDrive = -1 * pwmDrive;  
      ledcWrite(1, pwmDrive);  // channel 1 = DIR_PIN
      ledcWrite(0, 0);
      //digitalWrite(PWM_PIN, LOW);
    }
 }  

//---------------------------------------------------------------------
// Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
// Danfoss: PWM 50% On = Center Position
// Danfoss: PWM 75% On = Right Position max (above Valve=Center)
//---------------------------------------------------------------------
void motorDrive_IBT_Danfoss(void) 
  { 
    if (pwmDrive >  250) pwmDrive =  250; 
    if (pwmDrive < -250) pwmDrive = -250;
  
 /* if (steerEnable) digitalWrite(DIR_PIN, HIGH); // turn on /off Power
  else 
    {
      digitalWrite(DIR_PIN, LOW);  
      pwmDrive=0;
    }*/
  if (steerEnable) ledcWrite(1, 255);  // channel 1 = DIR_PIN // turn on /off Power
  else 
    {
      ledcWrite(1, 0);
      pwmDrive=0;
    }
    pwmDrive = pwmDrive / 4;  
    pwmOut = pwmDrive+ 128;  // add Center Pos.
    pwmDisplay = pwmOut;
    ledcWrite(0, pwmOut);  // channel 0 = PWM_PIN
  }


//--------------------------------------------------------------------
// Keya Motors http://www.dcmotorkeya.com/ manufactured till 2020
//---------------------------------------------------------------------
void motorDrive_Keya_v2020(void) 

  { 
    pwmDrive = pwmDrive * 4; // Gain is at about 30

    if (pwmDrive > 1000) pwmDrive = 1000;
    if (pwmDrive < -1000) pwmDrive = -1000;

    byte off[] = {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  };
    byte hold[] = {0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  };

    byte hi, lo;
    byte steer[] =   {0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  };

    if (pwmDrive > 0)
    {
      steer[4] = 0x00;
      steer[5] = 0x00;
    }
    else
    {
      steer[4] = 0xFF;
      steer[5] = 0xFF;
    }

    hi =  ( (pwmDrive) >> (8) ); // keep upper 8 bits
    lo = ( (pwmDrive) & (0xff) ); // keep lower 8 bits

    steer[6] = hi;
    steer[7] = lo;

   if (steerEnable == true && pwmDrive != 0) { 
    Serial.println(pwmDrive);
    UDPToWheel.beginPacket(steerSet.ipDestinationWheel, steerSet.portSendToWheel);
    UDPToWheel.write(steer, sizeof(steer));
    UDPToWheel.endPacket();
   }



// ********************** OFF *************************
   
   if (steerEnable == false) { 
    pwmDrive=0;
    Serial.println("off");
    UDPToWheel.beginPacket(steerSet.ipDestinationWheel, steerSet.portSendToWheel);
    UDPToWheel.write(off, sizeof(off));
    UDPToWheel.endPacket();
   }

// ********************** CENTER *************************

   if (steerEnable == true && pwmDrive == 0) { 
//   if (steerEnable == true && pwmDrive <= 1 && pwmDrive >= -1) { 
    Serial.println("hold");
    UDPToWheel.beginPacket(steerSet.ipDestinationWheel, steerSet.portSendToWheel);
    UDPToWheel.write(hold, sizeof(hold));
    UDPToWheel.endPacket();
    }

  }
 
