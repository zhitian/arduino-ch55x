/*
  ASCII table

  Prints out byte values in all possible formats:
  - as raw binary values
  - as ASCII-encoded decimal, hex, octal, and binary values

  For more on ASCII, see http://www.asciitable.com and http://en.wikipedia.org/wiki/ASCII

  The circuit: No external hardware needed.

  created 2006
  by Nicholas Zambetti
  modified 9 Apr 2012
  by Tom Igoe
  modified 28 Feb 2017 for use with sduino
  by Michael Mayer
  modified 13 Jun 2020
  by Deqing Sun for use with CH55xduino

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/ASCIITable
*/

void setup() {
  // No need to init USBSerial
  
  while (!USBSerial()) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // prints title with ending line break
  USBSerial_println_s("ASCII Table ~ Character Map");
}

// first visible ASCIIcharacter '!' is number 33:
int thisByte = 33;
// you can also write ASCII characters in single quotes.
// for example, '!' is the same as 33, so you could also use this:
// int thisByte = '!';

void loop() {
  // prints value unaltered, i.e. the raw binary version of the byte.
  // The Serial Monitor interprets all bytes as ASCII, so 33, the first number,
  // will show up as '!'
  USBSerial_write(thisByte);

  USBSerial_print_s(", dec: ");
  // prints value as string as an ASCII-encoded decimal (base 10).
  // Decimal is the default format for Serial.print() and Serial.println(),
  // so no modifier is needed:
  USBSerial_print_u(thisByte);
  // But you can declare the modifier for decimal if you want to.
  // this also works if you uncomment it:

  // Serial_print_ub(thisByte, DEC);


  USBSerial_print_s(", hex: ");
  // prints value as string in hexadecimal (base 16):
  USBSerial_print_ub(thisByte, HEX);

  USBSerial_print_s(", oct: ");
  // prints value as string in octal (base 8);
  USBSerial_print_ub(thisByte, OCT);

  USBSerial_print_s(", bin: ");
  // prints value as string in binary (base 2) also prints ending line break:
  USBSerial_println_ub(thisByte, BIN);

  // if printed last visible character '~' or 126, stop:
  if (thisByte == 126) {    // you could also use if (thisByte == '~') {
    // This loop loops forever and does nothing
    while (true) {
      continue;
    }
  }
  // go on to the next character
  thisByte++;
}
