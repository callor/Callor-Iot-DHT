/* DHT library 

MIT license
written by Adafruit Industries
*/

#include "DHT.h"

/**
  2017-04-12
  DHT ��ü�� ������
  �ɹ�ȣ�� ��������ȣ, ī��Ʈ ���� �ʱ� ������ �ѱ��

*/
DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
  _pin = pin;
  _type = type;
  _count = count;
  firstreading = true;
}

/**
 DHT ���� �������� HIGH�� �Ͽ� ������ �����Ѵ�

*/
void DHT::begin(void) {
  
  // set up the pins!
  pinMode(_pin, INPUT);
  digitalWrite(_pin, HIGH);
  _lastreadtime = 0;

}

/**
  �����κ��� �µ��б�
 
boolean S == Scale.  True == ȭ�� ; False == ����

*/
float DHT::readTemperature(bool S) {
  float f;

  if (read()) {
    switch (_type) {
    case DHT11:
      f = data[2];
      if(S)
      	f = convertCtoF(f);
      	
      return f;
    case DHT22:
    case DHT21:
      f = data[2] & 0x7F;
      f *= 256;
      f += data[3];
      f /= 10;
      if (data[2] & 0x80)
	f *= -1;
      if(S)
	f = convertCtoF(f);

      return f;
    }
  }
  return NAN;
}

// ������ ȭ���� ��ȯ
float DHT::convertCtoF(float c) {
	return c * 9 / 5 + 32;
}

// ȭ���� ������ ��ȯ
float DHT::convertFtoC(float f) {
  return (f - 32) * 5 / 9; 
}

/**
���� �б�

*/
float DHT::readHumidity(void) {
  float f;
  if (read()) {
    switch (_type) {
    case DHT11:
      f = data[0];
      return f;
    case DHT22:
    case DHT21:
      f = data[0];
      f *= 256;
      f += data[1];
      f /= 10;
      return f;
    }
  }
  return NAN;
}

/**
  ���� ���Ⱚ���� ���� 
  Heat Index ��°��� ���
  isFahrenheit: true �̸� ȭ��
  isFahrenheit: false �̸� ����
 */
float DHT::computeHeatIndex(float temperature, float percentHumidity, bool isFahrenheit) {
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  float hi;

  if (!isFahrenheit)
    temperature = convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temperature +
            10.14333127 * percentHumidity +
            -0.22475541 * temperature*percentHumidity +
            -0.00683783 * pow(temperature, 2) +
            -0.05481717 * pow(percentHumidity, 2) +
             0.00122874 * pow(temperature, 2) * percentHumidity +
             0.00085282 * temperature*pow(percentHumidity, 2) +
            -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return isFahrenheit ? hi : convertFtoC(hi);
}




/**
  ������ ���� ���� ���� �д� �Լ�
 */
boolean DHT::read(void) {
  uint8_t laststate = HIGH;
  uint8_t counter = 0;
  uint8_t j = 0, i;
  unsigned long currenttime;

  // ������ 2 �� �̳��� �ٽ� �������� Ȯ��
  currenttime = millis();
  if (currenttime < _lastreadtime) {
    // �������� ���� ��Ʈ Ÿ���� �ʱ�ȭ
    _lastreadtime = 0;
  }

  // 2000 - (currenttime - _lastreadtime)) : 2�ʸ��� �ѹ��� �������� ����
  if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
	return true; // ������ �����̸� true�� ����
  }

  // ���� ������ false�� ����
  firstreading = false;

  // ���� ���� ������ ���� �������� ����
  _lastreadtime = millis();

  // ������ �����迭 ����
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  

  // DHT ������ ������ ������ ��û�ϴ� �κ�
  
  // ���� ������ ���� 250ms�� HIGH�� ����
  digitalWrite(_pin, HIGH);
  delay(250);

  // �ɸ�带 ������� �����ϰ�, 20ms ���� LOW�� ����
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20);

  // ���ͷ�Ʈ ����
  //	���� �����͸� �д� ���� �Ƶ��̳뿡�� �����ϴ� 
  //	��� ���ͷ�Ʈ�� ��� ���ξ�� �Ѵ�
  //    �׷��� ������ ������ ���� �Ѿ�� Ÿ�̹� ���� ��ġ�� �ȴ�.
  noInterrupts();
  
  // 40uS ���� HIGH ���� �� �Է¸��� ����
  digitalWrite(_pin, HIGH);
  delayMicroseconds(40);
  pinMode(_pin, INPUT);

  // ������ ���� ���۵� �����͸� ì��� �κ�
  //    Ÿ�ֿ̹� �����ؾ� �Ѵ�

  for ( i=0; i< MAXTIMINGS; i++) {
    counter = 0;
    
	// ���� Ÿ�ֿ̹��� ���� ���� ���� ���� ������ 
	//		255uS ���� �б⸦ �ݺ��Ѵ�
	while (digitalRead(_pin) == laststate) {
      counter++;
      delayMicroseconds(1);
      if (counter == 255) {
        break;
      }
    }
    laststate = digitalRead(_pin);

	// �� 255 ��Ʈ�� ��� �о����� �ݺ����� ����
    if (counter == 255) break;

    // ó�� 3�� �ٲ� ���� ����
    if ((i >= 4) && (i%2 == 0)) {
      
	  // �� ��Ʈ�� ��� 1����Ʈ�� ����
      data[j/8] <<= 1;
      if (counter > _count)
        data[j/8] |= 1;
      j++;
    }
  }

  // ���ͷ�Ʈ �ٽ� ����
  interrupts();

  // 40��Ʈ�� �������� checksum�� �´��� Ȯ�� ������ true ����
  if ((j >= 40) && 
      (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) ) {
    return true;
  }
  return false;
}
