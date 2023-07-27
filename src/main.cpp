#include<Arduino.h>
#include<Wire.h>
#include<driver/i2c.h>
// Define accelerometer I2C address
#define KX132_I2C_ADDRESS 0x1F

//defining the I2C pins 
#define I2C_SDA 21
#define I2C_SCL 22

// I2C clock frequency (adjust as needed)
#define I2C_CLOCK_FREQ 100000

// Function prototypes
void i2cInit()               //Initialising the pins for I2C
{      
  // Set pins as outputs for I2C_SDA and I2C_SCL
  pinMode(I2C_SDA, OUTPUT);
  pinMode(I2C_SCL, OUTPUT);

}

void i2cStart()       //Giving Start Condition
{
  digitalWrite(I2C_SDA, HIGH);
  digitalWrite(I2C_SCL, HIGH);
  delayMicroseconds(2);
  digitalWrite(I2C_SDA, LOW);
  delayMicroseconds(2);
  digitalWrite(I2C_SCL, LOW);
}

void i2cStop()        //Giving Stop Condition 
{
  digitalWrite(I2C_SDA, LOW);
  delayMicroseconds(2);
  digitalWrite(I2C_SCL, HIGH);
  delayMicroseconds(2);
  digitalWrite(I2C_SDA, HIGH);
  delayMicroseconds(2);
}

void i2cSendByte(uint8_t byte)    //Sending the information Byte through SDA
{
  for (uint8_t i = 0; i < 8; ++i) {
    if (byte & 0x80)
      digitalWrite(I2C_SDA, HIGH);
    else
      digitalWrite(I2C_SDA, LOW);

    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(2);
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(2);
    byte <<= 1;                     //Shifting  1 bit to left
  }

  // Release I2C_SDA line for ACK
  digitalWrite(I2C_SDA, HIGH);
  digitalWrite(I2C_SCL, HIGH);
  delayMicroseconds(2);
  digitalWrite(I2C_SCL, LOW);
}

void writeRegister(uint8_t reg, uint8_t value)    //Writing value to a register with address reg
{
  i2cStart();
  i2cSendByte(KX132_I2C_ADDRESS << 1);
  i2cSendByte(reg);
  i2cSendByte(value);
  i2cStop();
}
uint8_t i2cReadByte(bool ack)       //Reading the information byte received through SDA 
{
  uint8_t byte = 0;

  pinMode(I2C_SDA, INPUT_PULLUP);

  for (uint8_t i = 0; i < 8; ++i) {
    byte <<= 1;                      //making LSB 0 to write
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(2);
    if (digitalRead(I2C_SDA))
      byte |= 0x01;                  //making LSB 1 to read
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(2);
  }

  // Send ACK or NACK
  digitalWrite(I2C_SDA, ack ? LOW : HIGH);
  digitalWrite(I2C_SCL, HIGH);
  delayMicroseconds(2);
  digitalWrite(I2C_SCL, LOW);

  pinMode(I2C_SDA, OUTPUT);
  digitalWrite(I2C_SDA, HIGH);

  return byte;
}

void initializeKX132()      //Initialising the accelerometer for I2C communication
{
  // Configure the accelerometer for +/-2g range and 10-bit resolution
  writeRegister(0x1B, 0x00); // CTRL_REG1: 0x41 (10-bit resolution, +/-2g range)
  writeRegister(0x21, 0x06);
  writeRegister(0x1B, 0xC0);

  // Wait for the accelerometer to settle
  delay(100);
}

int16_t readAxis(uint8_t regL, uint8_t regH)      //Reading the accelerometer data from the respective registers.
{
  int16_t axiI2C_sData;

  i2cStart();
  i2cSendByte(KX132_I2C_ADDRESS << 1);
  i2cSendByte(regL);
  i2cStop();

  i2cStart();
  i2cSendByte((KX132_I2C_ADDRESS << 1) | 1);
  axiI2C_sData = i2cReadByte(true);
  axiI2C_sData |= (static_cast<int16_t>(i2cReadByte(false)) << 8);
  i2cStop();

  return axiI2C_sData;
}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // Wait for serial port to connect

  // Initialize I2C communication
  i2cInit();

  // Initialize the accelerometer
  initializeKX132();

  Serial.println("KX132 accelerometer initialized!");
}

void loop() {
  // Read accelerometer data
  int16_t x = readAxis(0x08, 0x09);
  int16_t y = readAxis(0x0A, 0x0B);
  int16_t z = readAxis(0x0C, 0x0D);

  // Convert raw data to m/s^2
  float accelerationX = static_cast<float>(x) * 0.0078;
  float accelerationY = static_cast<float>(y) * 0.0078;
  float accelerationZ = static_cast<float>(z) * 0.0078;

  // Print the acceleration values in m/s^2
  Serial.print("X: "); Serial.print(accelerationX); Serial.print("  ");
  Serial.print("Y: "); Serial.print(accelerationY); Serial.print("  ");
  Serial.print("Z: "); Serial.print(accelerationZ); Serial.println(" m/s^2");

  delay(100); // Adjust the delay as needed
}

