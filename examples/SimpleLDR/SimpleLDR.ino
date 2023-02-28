// Simple-Helioponics-Arduino Simple Light Dependent Resistor (LDR) Example
//
// The Simple LDR Example sketch shows how a simple Helioduino system can be setup using
// the most minimal of work. In this sketch only that which you actually use is built
// into the final compiled binary, making it an ideal lean choice for those who don't
// need anything fancy. This sketch has no UI or input control, but with a simple buzzer
// and some additional sensors the system can control another axis for a gimballed system.
//
// LDR setups are great for beginners, and has the advantage of being able to be built out
// of commonly available materials. A positional servo is used to tilt the panel, which
// is mounted horizontally facing the general direction of the sun.

#include <Helioduino.h>

#define SETUP_PIEZO_BUZZER_PIN          -1              // Piezo buzzer pin, else -1
#define SETUP_AXIS_SERVO_PIN            A0              // Axis servo write pin (analog)
#define SETUP_LDR_LOWER_PIN             A1              // Lower LDR read pin (analog)
#define SETUP_LDR_UPPER_PIN             A2              // Upper LDR read pin (analog)

#define SETUP_SERVO_MIN_DEG             -90             // Minimum degrees of axial servo
#define SETUP_SERVO_MAX_DEG             90              // Maximum degrees of axial servo

#define SETUP_PANEL_TYPE                Horizontal      // Panel type (Horizontal, Vertical, Gimballed, Equatorial)
#define SETUP_PANEL_HOME                {0.0f,0.0f}     // Panel home position (azimuth,elevation or RA,declination)
#define SETUP_PANEL_OFFSET              {0.0f,0.0f}     // Panel offset position (azi,ele or RA,dec)

Helioduino helioController(SETUP_PIEZO_BUZZER_PIN);     // Controller using default setup aside from buzzer pin, if defined

float _SETUP_PANEL_HOME[] = SETUP_PANEL_HOME;
float _SETUP_PANEL_OFFSET[] = SETUP_PANEL_OFFSET;

void setup() {
    // Setup base interfaces
    #ifdef HELIO_ENABLE_DEBUG_OUTPUT
        Serial.begin(115200);           // Begin USB Serial interface
        while (!Serial) { ; }           // Wait for USB Serial to connect
    #endif

    // Initializes controller with LDR environment (saves some time/space), no logging, eeprom, SD, or anything else.
    helioController.init(Helio_SystemMode_Balancing);

    // Adds a simple horizontal LDR balanced solar panel, and sets up any specified offsets.
    auto panel = helioController.addLDRBalancingPanel(JOIN(Helio_PanelType,SETUP_PANEL_TYPE));
    panel->setHomePosition(_SETUP_PANEL_HOME);
    panel->setAxisOffset(_SETUP_PANEL_OFFSET);

    // Adds a simple positional servo at SETUP_AXIS_SERVO_PIN, installed to control the vertical elevation of the panel.
    auto axisServo = helioController.addPositionalServo(SETUP_AXIS_SERVO_PIN, SETUP_SERVO_MIN_DEG, SETUP_SERVO_MAX_DEG);
    axisServo->setParentPanel(panel, Helio_PanelAxis_Elevation);

    // Adds a light intensity sensor at SETUP_LDR_LOWER_PIN, installed on the lower side of the panel.
    auto ldrLower = helioController.addLightIntensitySensor(SETUP_LDR_LOWER_PIN);
    panel->setLDRSensor(ldrLower, Helio_PanelLDR_VerticalMin); // will provide downwards control

    // Adds a light intensity sensor at SETUP_LDR_UPPER_PIN, installed on the upper side of the panel.
    auto ldrUpper = helioController.addLightIntensitySensor(SETUP_LDR_UPPER_PIN);
    panel->setLDRSensor(ldrUpper, Helio_PanelLDR_VerticalMax); // will provide upwards control

    // Launches controller into main operation.
    helioController.launch();
}

void loop()
{
    // Helioduino will manage most updates for us.
    helioController.update();
}
