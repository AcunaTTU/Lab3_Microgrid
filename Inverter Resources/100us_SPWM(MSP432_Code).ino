// Lookup Table

int Time[ ] =
{
  600,
  200,
  300,
  200,
  300,
  200,
  200,
  400,
  100,
  400,
  100,
  400,
  100,
  400,
  100,
  400,
  100,
  900,
  100,
  400,
  100,
  400,
  100,
  400,
  100,
  400,
  100,
  400,
  200,
  200,
  300,
  200,
  300,
  200,
  700
};

int Array = 0;
int pin1 = 2;
int pin2 = 3;
int pin3 = 4;
int pin4 = 5;

int Control = 0;

int dead_time = 80;

int track = 0;

void setup() {
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);

  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, HIGH);
  digitalWrite(pin3, HIGH);
  digitalWrite(pin4, HIGH);


}

void loop() {

  delayMicroseconds(Time[Array] - dead_time);

  if (Control == 0)
  {

    if (Array == 35)
    {
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);

      Array = 0;
      track = 0;
      Control = 1;

      delayMicroseconds(dead_time);

      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, HIGH);
    }

    else if (track == 0)
    {
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);

      Array++;
      track = 1;

      delayMicroseconds(dead_time);

      digitalWrite(pin1, LOW);
      digitalWrite(pin2, LOW);
    }

    else if (track == 1)
    {
      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, LOW);

      Array++;
      track = 0;

      delayMicroseconds(dead_time);

      digitalWrite(pin1, HIGH);
      digitalWrite(pin2, HIGH);
    }

  }


  else
  {

    if (Array == 35)
    {
      digitalWrite(pin3, HIGH);
      digitalWrite(pin4, LOW);

      Array = 0;
      track = 0;
      Control = 0;

      delayMicroseconds(dead_time);

      digitalWrite(pin3, HIGH);
      digitalWrite(pin4, HIGH);
    }

    else if (track == 0)
    {
      digitalWrite(pin3, HIGH);
      digitalWrite(pin4, LOW);

      Array++;
      track = 1;

      delayMicroseconds(dead_time);

      digitalWrite(pin3, LOW);
      digitalWrite(pin4, LOW);
    }

    else if (track == 1)
    {
      digitalWrite(pin3, HIGH);
      digitalWrite(pin4, LOW);

      Array++;
      track = 0;

      delayMicroseconds(dead_time);

      digitalWrite(pin3, HIGH);
      digitalWrite(pin4, HIGH);
    }

  }

}
