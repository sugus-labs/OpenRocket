void Calculate_Events()
{
  if ( ! is_launched)
  {
    calLaunch();
  }
  else if ( ! descent && ! parachute)
  {
    calTrajSec();
    calDescent();
  }
  else if ( ! parachute)
  {
    dplParachute();
  }
  else if ( ! touchdown)
  {
    calTouchdown();
  }
}

void calLaunch()
{
  if (!is_launched && vector_module(accel) > 500) //TODO new values with new resolution
  {
    is_launched = true;
    launch = millis();
    Serial.println("Launch!!");
    //TODO change accelerometer resolution
  }
}

void calTrajSec()
{
  //TODO
}

void calDescent()
{
  if (!descent && millis() > launch + 5000)
  {
    descent = true;
    descentStart = millis();
  }
  //TODO
}

void dplParachute()
{
  if (!parachute && millis() > descentStart + 500)
  {
    myservo.write(180);
    parachute = true;
    parachuteDepl = millis();
  }
  //TODO
}

void calTouchdown()
{
  //TODO
}
