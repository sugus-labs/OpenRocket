/* This file is part of the Razor AHRS Firmware */

void output_sensors_text(char raw_or_calibrated)
{
//  Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
//  Serial.print(accel[0]*0.0428009); Serial.print(",");
//  Serial.print(accel[1]*0.0428009); Serial.print(",");
//  Serial.print(accel[2]*0.0428009); Serial.println();
//
//  Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
//  Serial.print(magnetom[0]); Serial.print(",");
//  Serial.print(magnetom[1]); Serial.print(",");
//  Serial.print(magnetom[2]); Serial.println();
//
//  Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
//  Serial.print(gyro[0]); Serial.print(",");
//  Serial.print(gyro[1]); Serial.print(",");
//  Serial.print(gyro[2]); Serial.println();
  if (dataFile) {
    dataFile.print(accel[0]*0.0428009); dataFile.print(",");
    dataFile.print(accel[1]*0.0428009); dataFile.print(",");
    dataFile.print(accel[2]*0.0428009); dataFile.print(",");
    dataFile.print(magnetom[0]); dataFile.print(",");
    dataFile.print(magnetom[1]); dataFile.print(",");
    dataFile.print(magnetom[2]); dataFile.print(",");
    dataFile.print(gyro[0]); dataFile.print(",");
    dataFile.print(gyro[1]); dataFile.print(",");
    dataFile.print(gyro[2]); dataFile.print(",");
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

}

void output_sensors()
{
  compensate_sensor_errors();
  output_sensors_text('C');
}

