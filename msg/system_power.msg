uint64 timestamp        # time since system start (microseconds)

bool voltage5v_available  # peripheral 5V rail voltage available
float32 voltage5v_v       # peripheral 5V rail voltage

bool[3] voltage3v3_available # sensor 3.3V rail voltage available
float32[3] voltage3v3_v      # sensor 3.3V rail voltage

bool usb_connected      # USB is connected when true
bool usb_power_valid    # On USB power when true

bool servo_power_available
bool servo_power_valid       # servo power is good when true

bool peripheral_5v_available     # 5V peripheral available
bool peripheral_5v_overcurrent   # 5V peripheral overcurrent when true

bool hipower_5v_available        # 5V hi power peripheral available
bool hipower_5v_overcurrent      # 5V hi power peripheral overcurrent when true

uint8 bricks_supported
bool[4] brick_valid     # brick power is good when true
