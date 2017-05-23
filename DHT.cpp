/* DHT library 

MIT license
written by Adafruit Industries
*/

#include "DHT.h"

/**
  2017-04-12
  DHT 객체를 생성자
  핀번호와 센서형번호, 카운트 값을 초기 값으로 넘긴다

*/
DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
  _pin = pin;
  _type = type;
  _count = count;
  firstreading = true;
}

/**
 DHT 센서 연결핀을 HIGH로 하여 센서를 시작한다

*/
void DHT::begin(void) {
  
  // set up the pins!
  pinMode(_pin, INPUT);
  digitalWrite(_pin, HIGH);
  _lastreadtime = 0;

}

/**
  센서로부터 온도읽기
 
boolean S == Scale.  True == 화씨 ; False == 섭씨

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

// 섭씨를 화씨로 변환
float DHT::convertCtoF(float c) {
	return c * 9 / 5 + 32;
}

// 화씨를 섭씨로 변환
float DHT::convertFtoC(float f) {
  return (f - 32) * 5 / 9; 
}

/**
습도 읽기

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
  센서 추출값으로 부터 
  Heat Index 출력값을 계산
  isFahrenheit: true 이면 화씨
  isFahrenheit: false 이면 섭씨
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
  센서로 부터 실제 값을 읽는 함수
 */
boolean DHT::read(void) {
  uint8_t laststate = HIGH;
  uint8_t counter = 0;
  uint8_t j = 0, i;
  unsigned long currenttime;

  // 센서가 2 초 이내에 다시 읽혔는지 확인
  currenttime = millis();
  if (currenttime < _lastreadtime) {
    // 읽힌적이 으면 라스트 타임을 초기화
    _lastreadtime = 0;
  }

  // 2000 - (currenttime - _lastreadtime)) : 2초마다 한번씩 측정값을 리턴
  if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
	return true; // 마지막 측정이면 true를 리턴
  }

  // 최초 읽음을 false로 설정
  firstreading = false;

  // 최종 읽은 시점을 현재 시점으로 설정
  _lastreadtime = millis();

  // 저장할 변수배열 설정
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;
  

  // DHT 센서에 데이터 전송을 요청하는 부분
  
  // 센서 데이터 핀을 250ms간 HIGH로 설정
  digitalWrite(_pin, HIGH);
  delay(250);

  // 핀모드를 출력으로 설정하고, 20ms 동안 LOW로 설정
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20);

  // 인터럽트 중지
  //	실제 데이터를 읽는 동안 아두이노에서 수행하는 
  //	모든 인터럽트를 잠시 꺼두어야 한다
  //    그렇지 않으면 센서로 부터 넘어온 타이밍 값을 놓치게 된다.
  noInterrupts();
  
  // 40uS 동안 HIGH 유지 후 입력모드로 설정
  digitalWrite(_pin, HIGH);
  delayMicroseconds(40);
  pinMode(_pin, INPUT);

  // 센서로 부터 전송된 데이터를 챙기는 부분
  //    타이밍에 주의해야 한다

  for ( i=0; i< MAXTIMINGS; i++) {
    counter = 0;
    
	// 이전 타이밍에서 읽은 값과 같은 값이 읽히면 
	//		255uS 동안 읽기를 반복한다
	while (digitalRead(_pin) == laststate) {
      counter++;
      delayMicroseconds(1);
      if (counter == 255) {
        break;
      }
    }
    laststate = digitalRead(_pin);

	// 총 255 비트를 모두 읽었으면 반복문을 종료
    if (counter == 255) break;

    // 처음 3번 바뀐 값은 무시
    if ((i >= 4) && (i%2 == 0)) {
      
	  // 각 비트를 모아 1바이트로 설정
      data[j/8] <<= 1;
      if (counter > _count)
        data[j/8] |= 1;
      j++;
    }
  }

  // 인터럽트 다시 수행
  interrupts();

  // 40비트가 읽혔으면 checksum이 맞는지 확인 맞으면 true 리턴
  if ((j >= 40) && 
      (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) ) {
    return true;
  }
  return false;
}
