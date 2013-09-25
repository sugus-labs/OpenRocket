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
  if (vector_module(accel) > 500) //TODO new values with new resolution
  {
    is_launched = true;
    Serial.println("Launch!!");
    //TODO change resolution
  }
}

void calTrajSec(){}
void calDescent(){}
void dplParachute(){}
void calTouchdown(){}
