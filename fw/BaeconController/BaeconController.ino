/**
 * Infrared controlled RGB beacon for musical beat.
 * 
 * This is a firmware for a RGB beacon device. The device is supposed to give
 * a musical beat to deaf people on dancing or signing lessons.
 * 
 * The device features IR remote controller for:
 * * turning on/off
 * * selecting beat mode
 * * (re)starting the sequence
 * * tuning the timing.
 * * Saving the cirrent settings into permanent memory.
 * 
 * The beacon gives the beat by a sequence of blinking colors, either mixed
 * together in one sequence, or separated with assumption that multiple beacons
 * are used. Each group is supposed to focus on their color, that gives them
 * the beat.
 * 
 * @author Michal Horn, Richard Gazik
 */

#define F_CPU 8000000
#include <IRremote.h>
#include <IRremoteInt.h>
#include <EEPROM.h>


/*****************************************************************
 *                  TYPES DEFINION                               *
 *****************************************************************/

/**
 * Enumeration of IR remote controller key codes, used in the applicatino.
 */
typedef enum {
  IR_KEY_UP = 0xFFA05F,
  IR_KEY_DOWN = 0xFF20DF,
  IR_KEY_OFF = 0xFF609F,
  IR_KEY_ON = 0xFFE01F,
  IR_KEY_R = 0xFF906F,
  IR_KEY_G = 0xFF10EF,
  IR_KEY_B = 0xFF50AF,
  IR_KEY_W = 0xFFD02F,
  IR_KEY_R_1 = 0xFFB04F,
  IR_KEY_R_2 = 0xFFA857,
  IR_KEY_R_3 = 0xFF9867,
  IR_KEY_G_1 = 0xFF30CF,
  IR_KEY_G_2 = 0xFF28D7,
  IR_KEY_G_3 = 0xFF18E7,
  IR_KEY_B_1 = 0xFF708F,
  IR_KEY_B_2 = 0xFF6897,
  IR_KEY_B_3 = 0xFF58A7,
  IR_KEY_B_4 = 0xFF48B7,
  IR_KEY_FLASH = 0xFFF00F,
  IR_NUM_KEYS = 19
} ir_key_codes_t;

/**
 * Beat modes enumeration.
 */
typedef enum {
  BEAT_MODE_1,
  BEAT_MODE_2,
  BEAT_MODE_3,
  BEAT_MODE_1_1,
  BEAT_MODE_1_2,
  BEAT_MODE_1_3,
  BEAT_MODE_2_1,
  BEAT_MODE_2_2,
  BEAT_MODE_2_3,
  BEAT_MODE_3_1,
  BEAT_MODE_3_2,
  BEAT_MODE_3_3,
  BEAT_MODE_3_4,
NUM_OF_MODES
} beat_mode_t;

/**
 * Application states.
 */
typedef enum {
  STATE_OFF,
  STATE_STOPPED,
  STATE_RUNNING
} state_t;

/**
 * Colors, defined by bits in byte. XXXX XBGR
 */
typedef enum {
  COLOR_OFF = 0x0,/**< LED OFF */
  COLOR_1 = 0x1,  /**< RED  */
  COLOR_2 = 0x2,  /**< GREEN */
  COLOR_3 = 0x4,  /**< BLUE */
  COLOR_4 = 0x5,  /**< R+B */
  COLOR_5 = 0x7,  /**< WHITE */
} color_t;

/**
 * Callback function pointer type definition.
 */
typedef void (ir_btn_cb_t)(unsigned int value);

/**
 * Structure mapping IR remote controller keys to callbacks and parameters.
 */
typedef struct {
  unsigned long key;      /**< IR remote controller code. */
  ir_btn_cb_t* callback;  /**< Pointer to callback, that is called for @ref given key with @ref param. */
  unsigned int param;     /**< Parameter to be passed to @ref callback. */
} key_map_t;

/*****************************************************************
 *                  GLOBAL VARIABLES                             *
 *****************************************************************/

IRrecv _receiver(3);          /**< IR receiver. */
decode_results _results;      /**< Variable storing the result of IR reception. */
unsigned long _beat_delay_ms; /**< The beat delay in milliseconds. */
beat_mode_t _beat_mode;       /**< The actual beating mode. */
unsigned int _beat_phase;    /**< The number of the beat in sequence. */
state_t _state;               /**< The application state. */
unsigned long _last_time_ms;  /**< Last time value, used by the timer. */

/*****************************************************************
 *                  FUNCTION DEFINITIONS                         *
 *****************************************************************/

/**
 * Increase the beat delay - decrease beat rate.
 * 
 * @param value The value to decrease the dealy.
 */
void increase_beat_delay(unsigned int value);

/**
 * Decrease the beat delay - increase beat rate.
 * 
 * @param value The value to increase the dealy.
 */
void decrease_beat_delay(unsigned int value);

/**
 * Set new beat mode.
 * 
 * After new beeat mode is set, it will start playing from the beginning.
 * 
 * @param mode The mode to set.
 * @see beat_mode_t
 * @see _mode_sequence
 */
void set_beat_mode(unsigned int mode);

/**
 * Start beat sequence playback.
 * 
 * @param param Not used.
 */
void start(unsigned int param);

/**
 * Turn the light off from any state.
 * 
 * @param param Not used.
 */
void turn_off(unsigned int param);

/**
 * Turn the beacon on.
 * 
 * The playback is stopped, but IR remote controller is being processed.
 * 
 * @param param Not used.
 */
void turn_on(unsigned int param);

/**
 * Set color on the RGB LED on.
 * 
 * @param color Color to be shown.
 * @see color_t
 */
void set_rgb_led(color_t color);

/**
 * See if given time duration since last @ref update_time call has elapsed.
 * 
 * @param duration Time to measure.
 * @return True if time has elapsed, False otherwise.
 */
bool time_elapsed(unsigned long duration);

/**
 * Get the next color from the beat sequence.
 * 
 * @param mode Current beat mode.
 * @param phase The current phase of the sequence.
 * @return Color of the mode on the phase.
 */
color_t get_signal(beat_mode_t mode, unsigned int phase);

/**
 * Go to next phase in the sequence, wrap at the beginning if the end was reached.
 * 
 * @param curr_phase Currenct phase.
 * @return Next phase number.
 */
unsigned int next_signal(unsigned int curr_phase);

/**
 * Fetch current time from timer.
 */
void update_time();

/**
 * Handle key code.
 * 
 * Calls callbacks registered for given key code.
 * 
 * @param key_code Key code.
 */
void handle_key(unsigned long key_code);

/**
 * Simple callback to invoke settings save function with more parameters.
 * 
 * @param param Not used.
 */
void save_settings_cb(unsigned int param);

/**
 * Store settings to EEPROM.
 * 
 * @param delay)ms The beat delay to be saved.
 * @param mode The beat mode tobe saved.
 */
void save_settings(unsigned long delay_ms, uint8_t mode);

/**
 * Load settings to EEPROM.
 * 
 * @param delay_ms Where the beat delay will be loaded.
 * @param mode Where the beat mode will be loaded.
 * 
 * @return true if settings loaded, false otherwise. In this
 * case the loaded data are invalid and should not be used.
 */
bool load_settings(unsigned long* delay_ms, uint8_t* mode);

/**
 * Compute a Dallas Semiconductor 8 bit CRC
 * 
 * @param addr Starting data address.
 * @param len The length of data.
 * 
 * @note Taken from https://github.com/PaulStoffregen/OneWire/
 */
static uint8_t crc8(const uint8_t *addr, uint8_t len);

/*****************************************************************
 *                  CONSTANTS                                    *
 *****************************************************************/
 
const unsigned long BLINK_DURATION_MS = 50;
const unsigned long INITIAL_BEAT_DELAY_MS = 250;
const unsigned long BEAT_DELAY_INC_MS = 25;
const unsigned long MINIMAL_BEAT_DELAY_MS = BEAT_DELAY_INC_MS;
const unsigned long MAXIMAL_BEAT_DELAY_MS = 2500;
const beat_mode_t INITIAL_SIG_MODE = BEAT_MODE_1;
const unsigned int NUM_OF_BEATS = 8;
const unsigned int EEPROM_RAW_DATA_LEN = 6;

const key_map_t _key_mapping[IR_NUM_KEYS] = {
  {.key = IR_KEY_UP,   .callback = increase_beat_delay, .param = BEAT_DELAY_INC_MS},
  {.key = IR_KEY_DOWN, .callback = decrease_beat_delay, .param = BEAT_DELAY_INC_MS},
  {.key = IR_KEY_ON,   .callback = turn_on,  .param = 0},
  {.key = IR_KEY_OFF,  .callback = turn_off, .param = 0},
  {.key = IR_KEY_W,    .callback = start,   .param = 0},
  {.key = IR_KEY_R,    .callback = set_beat_mode, .param = BEAT_MODE_1},
  {.key = IR_KEY_G,    .callback = set_beat_mode, .param = BEAT_MODE_2},
  {.key = IR_KEY_B,    .callback = set_beat_mode, .param = BEAT_MODE_3},
  {.key = IR_KEY_R_1,    .callback = set_beat_mode, .param = BEAT_MODE_1_1},
  {.key = IR_KEY_R_2,    .callback = set_beat_mode, .param = BEAT_MODE_1_2},
  {.key = IR_KEY_R_3,    .callback = set_beat_mode, .param = BEAT_MODE_1_3},
  {.key = IR_KEY_G_1,    .callback = set_beat_mode, .param = BEAT_MODE_2_1},
  {.key = IR_KEY_G_2,    .callback = set_beat_mode, .param = BEAT_MODE_2_2},
  {.key = IR_KEY_G_3,    .callback = set_beat_mode, .param = BEAT_MODE_2_3},
  {.key = IR_KEY_B_1,    .callback = set_beat_mode, .param = BEAT_MODE_3_1},
  {.key = IR_KEY_B_2,    .callback = set_beat_mode, .param = BEAT_MODE_3_2},
  {.key = IR_KEY_B_3,    .callback = set_beat_mode, .param = BEAT_MODE_3_3},
  {.key = IR_KEY_B_4,    .callback = set_beat_mode, .param = BEAT_MODE_3_4},
  {.key = IR_KEY_FLASH,  .callback = save_settings_cb, .param = 0},
};

const color_t _mode_sequence[NUM_OF_MODES][NUM_OF_BEATS] = {
  /* Combined beats. */
  {COLOR_1, COLOR_2, COLOR_2, COLOR_2, COLOR_3, COLOR_2, COLOR_2, COLOR_2},
  {COLOR_1, COLOR_1, COLOR_1, COLOR_3, COLOR_1, COLOR_1, COLOR_1, COLOR_2},
  {COLOR_1, COLOR_3, COLOR_3, COLOR_2, COLOR_4, COLOR_3, COLOR_3, COLOR_2},
  /* Separated beats mode 1 */
  {COLOR_1, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF},
  {COLOR_OFF, COLOR_2, COLOR_2, COLOR_2, COLOR_OFF, COLOR_2, COLOR_2, COLOR_2},
  {COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_3, COLOR_OFF, COLOR_OFF, COLOR_OFF},
  /* Separated beats mode 2 */
  {COLOR_1, COLOR_1, COLOR_1, COLOR_OFF, COLOR_1, COLOR_1, COLOR_1, COLOR_OFF},
  {COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_2},
  {COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_3, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF},
  /* Separated beats mode 3 */
  {COLOR_1, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF},
  {COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_2, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_2},
  {COLOR_OFF, COLOR_3, COLOR_3, COLOR_OFF, COLOR_OFF, COLOR_3, COLOR_3, COLOR_OFF},
  {COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_OFF, COLOR_4, COLOR_OFF, COLOR_OFF, COLOR_OFF},
};

/*****************************************************************
 *                  MAIN PROGRAMM                                *
 *****************************************************************/

void setup() {
  pinMode(0, OUTPUT);  /* R */
  pinMode(1, OUTPUT);  /* G */
  pinMode(2, OUTPUT);  /* B */
  _receiver.enableIRIn();
  
  turn_off(0);
}

void loop() {
  /* Process IR remote controller events. */
  if(_receiver.decode(&_results)) {
    handle_key(_results.value);
    _receiver.resume();
  }

  /* Process the beats lightning */
  if (_state == STATE_RUNNING) {
    if (time_elapsed(_beat_delay_ms)) {
      color_t color = get_signal(_beat_mode, _beat_phase);
      set_rgb_led(color);
      delay(BLINK_DURATION_MS);
      set_rgb_led(COLOR_OFF);
      update_time();
      _beat_phase = next_signal(_beat_phase);
    }
  }
}

/*****************************************************************
 *                  FUNCTIONS DECLARATIONS                       *
 *****************************************************************/

void increase_beat_delay(unsigned int increase) {
  if (_state == STATE_OFF) {
    return;
  }
    
  if (_beat_delay_ms < MAXIMAL_BEAT_DELAY_MS) {
    _beat_delay_ms += increase;
  }
  update_time();
}

void decrease_beat_delay(unsigned int decrease) {
  if (_state == STATE_OFF) {
    return;
  }
  
  if (_beat_delay_ms > MINIMAL_BEAT_DELAY_MS) {
    _beat_delay_ms -= decrease;
  }
  update_time();
}

void set_beat_mode(unsigned int mode) {
  if (_state == STATE_OFF) {
    return;
  }

  for (int i = 0; i < mode+1; i++) {
    set_rgb_led(COLOR_1);
    delay(BLINK_DURATION_MS);
    set_rgb_led(COLOR_OFF);
    delay(BLINK_DURATION_MS);
  } 
  
  _beat_mode = mode;
  _beat_phase = 0;
  update_time();
}

void start(unsigned int value) {
  if (_state == STATE_OFF) {
    return;
  }
 
  set_rgb_led(COLOR_5);
  delay(BLINK_DURATION_MS);
  set_rgb_led(COLOR_OFF);
  delay(BLINK_DURATION_MS);
  set_rgb_led(COLOR_5);
  delay(BLINK_DURATION_MS);
  set_rgb_led(COLOR_OFF);
  delay(2*BLINK_DURATION_MS);

 _state = STATE_RUNNING;
  _beat_phase = 0;
  update_time();
}

void turn_off(unsigned int value) {
  set_rgb_led(COLOR_OFF);
  _state = STATE_OFF;
}

void turn_on(unsigned int value) {
  if (_state != STATE_OFF) {
    return;
  }
  
  set_rgb_led(COLOR_5);
  delay(BLINK_DURATION_MS);
  set_rgb_led(COLOR_OFF);
  if (!load_settings(&_beat_delay_ms, (uint8_t*)&_beat_mode)) {
    set_rgb_led(COLOR_1);
    delay(2*BLINK_DURATION_MS);
    set_rgb_led(COLOR_OFF);
    _beat_delay_ms = INITIAL_BEAT_DELAY_MS;
    _beat_mode = INITIAL_SIG_MODE;    
  }
  _state = STATE_STOPPED;
}

void set_rgb_led(color_t color) {
  digitalWrite(0, (color & 0x1) ? HIGH : LOW);
  digitalWrite(1, (color & 0x2) ? HIGH : LOW);
  digitalWrite(2, (color & 0x4) ? HIGH : LOW);
}

bool time_elapsed(unsigned long duration) {
  return (millis() - _last_time_ms >= duration);  
}

color_t get_signal(beat_mode_t mode, unsigned int phase) {
  color_t ret_color = COLOR_OFF;
  if (mode < NUM_OF_MODES && phase < NUM_OF_BEATS) {
    ret_color = _mode_sequence[mode][phase];
  }
  return ret_color;
}

unsigned int next_signal(unsigned int curr_phase) {
  return (curr_phase+1)%NUM_OF_BEATS;
}

void update_time() {
  _last_time_ms = millis();
}

void handle_key(unsigned long key_code) {
  for (int i = 0; i < IR_NUM_KEYS; i++) {
    if (_key_mapping[i].key == key_code) {
      _key_mapping[i].callback(_key_mapping[i].param);
    }
  }
}

void save_settings_cb(unsigned int param) {
  save_settings(_beat_delay_ms, _beat_mode);
}

void save_settings(unsigned long delay_ms, uint8_t mode) {
  uint8_t raw_data[EEPROM_RAW_DATA_LEN] = {
    (uint8_t)(delay_ms), 
    (uint8_t)(delay_ms>>8),
    (uint8_t)(delay_ms>>16), 
    (uint8_t)(delay_ms>>24),
    mode,
    0
  };

  raw_data[5] = crc8(raw_data, EEPROM_RAW_DATA_LEN-1);

  for (int i = 0; i < EEPROM_RAW_DATA_LEN; i++) {
    EEPROM.write(i, raw_data[i]);
  }

  set_rgb_led(COLOR_5);
  delay(BLINK_DURATION_MS);
  set_rgb_led(COLOR_OFF);
  delay(BLINK_DURATION_MS);
  set_rgb_led(COLOR_5);
  delay(BLINK_DURATION_MS);
  set_rgb_led(COLOR_OFF);
}

bool load_settings(unsigned long* delay_ms, uint8_t* mode) {
  uint8_t raw_data[EEPROM_RAW_DATA_LEN];

  for (int i = 0; i < EEPROM_RAW_DATA_LEN; i++) {
    raw_data[i] = EEPROM.read(i);
  }

  uint8_t crc = crc8(raw_data, EEPROM_RAW_DATA_LEN-1);
  if (crc != raw_data[EEPROM_RAW_DATA_LEN-1]) {
    return false;
  }

  *delay_ms = (uint32_t)raw_data[0] | (raw_data[1]<<8) | (raw_data[2]<<16) | (raw_data[3]<<24);
  *mode = raw_data[4];
  return true;
}

// Dow-CRC using polynomial X^8 + X^5 + X^4 + X^0
// Tiny 2x16 entry CRC table created by Arjen Lentz
// See http://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t PROGMEM dscrc2x16_table[] = {
  0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
  0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
  0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
  0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};

uint8_t crc8(const uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;

  while (len--) {
    crc = *addr++ ^ crc;  // just re-using crc as intermediate
    crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
    pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
  }

  return crc;
}
