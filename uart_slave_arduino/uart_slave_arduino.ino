uint8_t RxBuffer[7];
uint8_t TxBuffer[7];
uint8_t i;
uint8_t Rx;
uint8_t k;
uint8_t command;
uint8_t dataMSB;
uint8_t dataLSB;
uint8_t checksum_received;
uint8_t checksum_transmitted;
uint16_t distance;
const int trig = 9;
const int echo = 10;
void setup() 
{
Serial.begin(115200);
Serial1.begin(115200); //slave uart
pinMode(6, OUTPUT);
pinMode(trig, OUTPUT);
pinMode(echo, INPUT);
digitalWrite(6, LOW);
}

uint16_t ReadSensor()
{
 digitalWrite(trig, LOW);
 delayMicroseconds(2);
 digitalWrite(trig, HIGH);
 delayMicroseconds(10);
 digitalWrite(trig, LOW);
 long time = pulseIn(echo, HIGH);
 distance = (time / 2) / 29.1;
 Serial.print("Distance:  ");
 Serial.print(distance);
 Serial.println(" cm");
 delay(100);
 return distance;  
}

bool isValidData(uint8_t *buffer)
{
if(buffer[0] != 0xFF)
return false;

if(buffer[1] != 0x01) // adress of the arduino slave
return false;

if(buffer[2] != 0x00)
return false;

checksum_received = buffer[1] + buffer[2] + buffer[3] + buffer[4] + buffer[5];

if(buffer[6] != checksum_received)
return false;

return true;
}

void HandleData(uint8_t *buffer)
{
  command = buffer[3];
  dataMSB = buffer[4];
  dataLSB = buffer[5];

  switch(command)
  {
    case 0x2F:
    ReadSensor();
    dataMSB = (distance >> 8) & 0xFF;
    dataLSB = distance & 0xFF;
    TxBuffer[0] = 0xFF;
    TxBuffer[1] = 0x01;
    TxBuffer[2] = 0x00;
    TxBuffer[3] = 0x27;
    TxBuffer[4] = dataMSB;
    TxBuffer[5] = dataLSB;
    checksum_transmitted = TxBuffer[1] + TxBuffer[2] + TxBuffer[3] + TxBuffer[4] + TxBuffer[5];
    TxBuffer[6] = checksum_transmitted;
    digitalWrite(6, HIGH);
    Serial1.write(TxBuffer, 7);
    delay(300);
    digitalWrite(6, LOW);
    Serial.println("Cevap Gönderildi");
    break;
    
    default:

    break;
  }
  
}

void loop() 
{ 
 if(Serial1.available() > 0)
 {

  Rx = Serial1.read();
  RxBuffer[i] = Rx;
  i++;

  if(i == 7)
  {
  Serial.println("Paket alındı:");
  i = 0;
  for(k = 0; k <= 6; k++)
  {
  Serial.println(RxBuffer[k], HEX);
  }
  if(isValidData(RxBuffer))
  { 
    Serial.println("Data Geçerli");
    HandleData(RxBuffer);
  }
  } 
}
}
