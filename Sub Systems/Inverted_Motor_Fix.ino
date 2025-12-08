// Intake Motor
if (PestoLink.buttonHeld(1) || PestoLink.buttonHeld(2) || PestoLink.keyHeld(Key::A) || PestoLink.keyHeld(Key::B)) {
  if (PestoLink.buttonHeld(2) || PestoLink.keyHeld(Key::B)) { // Ts so arbitrary 😭
    myMotor.setInverted(false);
  } else if (PestoLink.buttonHeld(1) || PestoLink.keyHeld(Key::A)) {
    myMotor.setInverted(true);
  }
  myMotor.set(1);
} else {
  myMotor.set(0);
}
