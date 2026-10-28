#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Preferences.h>
#include <math.h>

#define TFT_CS 5
#define TFT_DC 2

#define ENC_CLK 32
#define ENC_DT 33
#define ENC_SW 25

Adafruit_ILI9341 tft(TFT_CS, TFT_DC);

Preferences prefs;
bool prefsReady = false;

const uint16_t BG = ILI9341_BLACK;
const uint16_t CARD = 0x1082;
const uint16_t GRID = 0x4208;
const uint16_t WHITE = ILI9341_WHITE;
const uint16_t YELLOW = ILI9341_YELLOW;

const double RAD = PI / 180.0;
const double EARTH_KM = 6371.0088;

const double DEMO_LAT = 16.7050;
const double DEMO_LON = 74.2433;
const double RANGE_KM = 50.0;

// Fictional sample flights.
// These are not live positions or real routes.
struct Aircraft {
  const char *number;
  const char *airline;
  const char *model;
  const char *from;
  const char *to;

  double latitude;
  double longitude;

  uint32_t altitudeFt;
  int speedKmh;
  float trackDeg;
};

Aircraft aircraft[] = {
  {"AI657", "Air India", "Airbus A320",
   "Mumbai (BOM)", "Delhi (DEL)",
   0, 0, 34000, 823, 47},

  {"6E204", "IndiGo", "Airbus A320",
   "Pune (PNQ)", "Bengaluru (BLR)",
   0, 0, 32000, 790, 127},

  {"SG467", "SpiceJet", "Boeing 737",
   "Delhi (DEL)", "Goa (GOI)",
   0, 0, 28000, 745, 249},

  {"AI101", "Air India", "Boeing 787",
   "Delhi (DEL)", "New York (JFK)",
   0, 0, 36000, 850, 335},

  {"6E611", "IndiGo", "Airbus A321",
   "Hyderabad (HYD)", "Mumbai (BOM)",
   0, 0, 30000, 780, 85},

  {"QP128", "Akasa Air", "Boeing 737 MAX",
   "Mumbai (BOM)", "Chennai (MAA)",
   0, 0, 33000, 810, 190},

  {"IX153", "Air India Express", "Boeing 737",
   "Kochi (COK)", "Dubai (DXB)",
   0, 0, 29000, 760, 285}
};

const int PLANE_COUNT =
    sizeof(aircraft) / sizeof(aircraft[0]);

const double demoDistances[] = {
  8.2, 16.4, 22.7, 27.5, 35.1, 41.4, 48.3
};

double observerLat = DEMO_LAT;
double observerLon = DEMO_LON;

double distances[PLANE_COUNT];
double bearings[PLANE_COUNT];

int visible[PLANE_COUNT];
int visibleCount = 0;

enum Page {
  RADAR,
  DETAILS,
  SETTINGS,
  LOCATION,
  WIFI_PAGE
};

Page page = RADAR;

// The selection after the last aircraft is Settings.
int radarCursor = 0;
int selectedAircraft = 0;
int detailsPage = 0;
int selectedSetting = 0;

bool redraw = true;
String notice = "";

// Location editing modes:
// 0 = choose Latitude, Longitude, Save or Cancel
// 1 = choose a coordinate digit or Done
// 2 = change the selected digit
int editRow = 0;
int editMode = 0;
int editDigit = 3;

int32_t magnitude[2];
bool negativeCoord[2];

// Character positions in "+016.7050".
// The decimal point is skipped.
// Selection index 8 is the Done button.
const int digitPosition[] = {
  0, 1, 2, 3, 5, 6, 7, 8
};

const int32_t digitPlace[] = {
  0, 1000000, 100000, 10000, 1000, 100, 10, 1
};

volatile int encoderSteps = 0;
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;

bool lastReading = HIGH;
bool buttonState = HIGH;
bool holdHandled = false;

uint32_t changedAt = 0;
uint32_t pressedAt = 0;

// ---------- Encoder ----------

void IRAM_ATTR readEncoder() {
  portENTER_CRITICAL_ISR(&encoderMux);

  encoderSteps += digitalRead(ENC_DT) == HIGH ? 1 : -1;

  portEXIT_CRITICAL_ISR(&encoderMux);
}

int wrap(int value, int count) {
  return (value % count + count) % count;
}

// ---------- Display helpers ----------

void textAt(int x, int y, const String &s,
            int size = 1, uint16_t color = WHITE) {
  tft.setTextColor(color);
  tft.setTextSize(size);
  tft.setCursor(x, y);
  tft.print(s);
}

void frame(const char *title) {
  tft.fillScreen(BG);
  textAt(8, 8, title, 2);
  tft.fillRoundRect(5, 39, 230, 247, 12, CARD);
}

String coordinateText(double value, bool latitude) {
  return String(fabs(value), 4) + " " +
         (latitude
              ? (value < 0 ? "S" : "N")
              : (value < 0 ? "W" : "E"));
}

String bearingText(double value) {
  const char *compass[] = {
    "N", "NE", "E", "SE", "S", "SW", "W", "NW"
  };

  int degrees = ((int)lround(value)) % 360;

  return String(compass[((degrees + 22) / 45) % 8]) +
         " " + String(degrees) + " deg";
}

// ---------- Geographic calculations ----------

// Aircraft positions remain fixed when the observer location changes.
void seedDemoPositions() {
  double lat = DEMO_LAT * RAD;
  double lon = DEMO_LON * RAD;

  for (int i = 0; i < PLANE_COUNT; i++) {
    double angle = aircraft[i].trackDeg * RAD;
    double arc = demoDistances[i] / EARTH_KM;

    double pLat = asin(
        sin(lat) * cos(arc) +
        cos(lat) * sin(arc) * cos(angle));

    double pLon = lon + atan2(
        sin(angle) * sin(arc) * cos(lat),
        cos(arc) - sin(lat) * sin(pLat));

    aircraft[i].latitude = pLat / RAD;
    aircraft[i].longitude =
        fmod(pLon / RAD + 540.0, 360.0) - 180.0;
  }
}

void updateNearby() {
  visibleCount = 0;

  double lat1 = observerLat * RAD;

  for (int i = 0; i < PLANE_COUNT; i++) {
    double lat2 = aircraft[i].latitude * RAD;

    double dLat = lat2 - lat1;
    double dLon =
        (aircraft[i].longitude - observerLon) * RAD;

    double h =
        pow(sin(dLat / 2), 2) +
        cos(lat1) * cos(lat2) * pow(sin(dLon / 2), 2);

    h = fmax(0.0, fmin(1.0, h));

    distances[i] =
        2 * EARTH_KM * atan2(sqrt(h), sqrt(1 - h));

    double angle = atan2(
        sin(dLon) * cos(lat2),
        cos(lat1) * sin(lat2) -
        sin(lat1) * cos(lat2) * cos(dLon));

    bearings[i] =
        fmod(angle / RAD + 360.0, 360.0);

    if (distances[i] <= RANGE_KM) {
      visible[visibleCount++] = i;
    }
  }

  // Sort visible aircraft from nearest to farthest.
  for (int i = 1; i < visibleCount; i++) {
    int id = visible[i];
    int j = i - 1;

    while (j >= 0 &&
           distances[visible[j]] > distances[id]) {
      visible[j + 1] = visible[j];
      j--;
    }

    visible[j + 1] = id;
  }

  radarCursor = 0;
}

// ---------- Icons ----------

void drawGear(bool selected) {
  uint16_t color = selected ? YELLOW : WHITE;

  if (selected) {
    tft.drawRoundRect(201, 3, 31, 31, 5, color);
  }

  for (int i = 0; i < 8; i++) {
    double a = i * PI / 4;

    tft.drawLine(
        216 + lround(sin(a) * 8),
        18 - lround(cos(a) * 8),
        216 + lround(sin(a) * 11),
        18 - lround(cos(a) * 11),
        color);
  }

  tft.drawCircle(216, 18, 8, color);
  tft.drawCircle(216, 18, 3, color);
}

void planeIcon(int x, int y, float heading,
               uint16_t color) {
  double s = sin(heading * RAD);
  double c = cos(heading * RAD);

  const int px[] = {0, 0, -7, 7, -4, 4};
  const int py[] = {-8, 8, 1, 1, 6, 6};

  int sx[6];
  int sy[6];

  for (int i = 0; i < 6; i++) {
    sx[i] = x + lround(px[i] * c - py[i] * s);
    sy[i] = y + lround(px[i] * s + py[i] * c);
  }

  tft.drawLine(sx[0], sy[0], sx[1], sy[1], color);
  tft.drawLine(sx[2], sy[2], sx[3], sy[3], color);
  tft.drawLine(sx[4], sy[4], sx[5], sy[5], color);
}

// ---------- Radar screen ----------

void drawRadar() {
  tft.fillScreen(BG);

  textAt(8, 8, "RADAR", 2);
  textAt(86, 14, "DEMO", 1, YELLOW);

  bool gearSelected = radarCursor == visibleCount;
  drawGear(gearSelected);

  tft.fillRoundRect(5, 39, 230, 208, 12, CARD);

  const int cx = 120;
  const int cy = 143;
  const int radius = 82;

  for (int r = 27; r <= radius; r += 27) {
    tft.drawCircle(cx, cy, r, GRID);
  }

  tft.drawLine(cx - radius, cy, cx + radius, cy, GRID);
  tft.drawLine(cx, cy - radius, cx, cy + radius, GRID);

  textAt(117, 46, "N");
  textAt(218, 140, "E");
  textAt(17, 140, "W");
  textAt(117, 232, "S");

  textAt(15, 232, "Range: 50 km");
  textAt(175, 232, String(visibleCount) + " planes");

  // Draw the selected aircraft last.
  for (int pass = 0; pass < 2; pass++) {
    for (int n = 0; n < visibleCount; n++) {
      bool selected =
          !gearSelected && n == radarCursor;

      if (selected != (pass == 1)) continue;

      int i = visible[n];

      double a = bearings[i] * RAD;
      double r = distances[i] * radius / RANGE_KM;

      int x = cx + lround(sin(a) * r);
      int y = cy - lround(cos(a) * r);

      if (selected) {
        tft.drawCircle(x, y, 11, YELLOW);
      }

      planeIcon(
          x, y, aircraft[i].trackDeg,
          selected ? YELLOW : WHITE);
    }
  }

  // Center dot represents the tracker.
  tft.fillCircle(cx, cy, 3, WHITE);

  if (gearSelected) {
    textAt(8, 258, "Settings", 2, YELLOW);

    textAt(
        8, 278,
        visibleCount
            ? "Manual location / Wi-Fi"
            : "No aircraft within 50 km");
  } else {
    int i = visible[radarCursor];

    textAt(
        8, 258,
        String(aircraft[i].number) + "  " +
            aircraft[i].airline,
        1, YELLOW);

    textAt(
        8, 276,
        String(distances[i], 1) + " km | " +
            bearingText(bearings[i]));
  }

  textAt(8, 298, "Turn: planes + Settings");
  textAt(8, 311, "Press: open | Hold: back");
}

// ---------- Aircraft details ----------

void detailField(int y, const char *label,
                 const String &value) {
  textAt(17, y, label);
  textAt(17, y + 14, value, 2);
}

void drawDetails() {
  const Aircraft &a = aircraft[selectedAircraft];

  frame(a.number);

  textAt(
      180, 14,
      String("DEMO ") + String(detailsPage + 1) + "/2",
      1, YELLOW);

  if (detailsPage == 0) {
    detailField(49, "Airline / name", a.airline);
    detailField(89, "Aircraft", a.model);
    detailField(129, "From", a.from);
    detailField(169, "To", a.to);

    textAt(
        17, 217,
        "Lat: " + coordinateText(a.latitude, true));

    textAt(
        17, 239,
        "Lon: " + coordinateText(a.longitude, false));

    textAt(
        17, 267,
        "Sample route and position",
        1, YELLOW);
  } else {
    detailField(
        49, "Altitude",
        String(a.altitudeFt) + " ft");

    detailField(
        97, "Ground speed",
        String(a.speedKmh) + " km/h");

    detailField(
        145, "Distance from tracker",
        String(distances[selectedAircraft], 1) + " km");

    detailField(
        193, "Bearing from tracker",
        bearingText(bearings[selectedAircraft]));

    detailField(
        241, "Ground track",
        String(a.trackDeg, 0) + " deg");
  }

  textAt(8, 298, "Turn: other details page");
  textAt(8, 311, "Press / hold: radar");
}

// ---------- Settings screen ----------

void drawSettings() {
  frame("Settings");

  const char *items[] = {
    "Manual location", "Wi-Fi", "Back"
  };

  for (int i = 0; i < 3; i++) {
    int y = 69 + i * 46;

    if (i == selectedSetting) {
      tft.fillRoundRect(
          12, y - 5, 216, 28, 5, GRID);
    }

    textAt(
        16, y,
        String(i == selectedSetting ? "> " : "  ") +
            items[i],
        2,
        i == selectedSetting ? YELLOW : WHITE);
  }

  textAt(
      17, 212,
      "Lat: " + coordinateText(observerLat, true));

  textAt(
      17, 233,
      "Lon: " + coordinateText(observerLon, false));

  textAt(
      17, 267,
      notice.length()
          ? notice
          : "Location controls radar center");

  textAt(8, 298, "Turn: option | Press: open");
  textAt(8, 311, "Hold: radar");
}

// ---------- Manual location screen ----------

String editableCoordinate(int row) {
  char buffer[16];

  snprintf(
      buffer, sizeof(buffer),
      "%c%03ld.%04ld",
      negativeCoord[row] ? '-' : '+',
      (long)(magnitude[row] / 10000),
      (long)(magnitude[row] % 10000));

  return String(buffer);
}

void drawLocation() {
  frame("Location");

  for (int row = 0; row < 2; row++) {
    int y = 75 + row * 68;
    bool active = row == editRow;

    textAt(
        17, y - 20,
        row == 0
            ? "Latitude (-90 to +90)"
            : "Longitude (-180 to +180)");

    if (active) {
      tft.drawRoundRect(
          12, y - 5, 216, 29, 5, YELLOW);
    }

    String value = editableCoordinate(row);
    textAt(28, y, value, 2);

    if (active && editMode > 0 && editDigit < 8) {
      int x = 28 + digitPosition[editDigit] * 12;

      if (editMode == 2) {
        tft.fillRect(x, y - 2, 12, 20, YELLOW);

        textAt(
            x, y,
            String(value[digitPosition[editDigit]]),
            2, BG);
      } else {
        tft.drawLine(
            x, y + 18, x + 10, y + 18, YELLOW);
      }
    }

    bool doneSelected =
        active && editMode == 1 && editDigit == 8;

    tft.fillRoundRect(
        165, y - 4, 51, 25, 4,
        doneSelected ? YELLOW : GRID);

    textAt(
        178, y + 5, "Done", 1,
        doneSelected ? BG : WHITE);
  }

  for (int i = 2; i < 4; i++) {
    int y = 216 + (i - 2) * 37;

    if (editRow == i) {
      tft.fillRoundRect(
          12, y - 5, 216, 28, 5, GRID);
    }

    textAt(
        17, y,
        i == 2 ? "Save" : "Cancel",
        2,
        editRow == i ? YELLOW : WHITE);
  }

  textAt(
      8, 298,
      editMode == 0
          ? "Turn: up/down | Press: open"
          : editMode == 1
              ? "Turn: digit or Done"
              : "Turn: change value");

  textAt(
      8, 311,
      editMode == 0
          ? "Choose a row, Save or Cancel"
          : editMode == 1
              ? "Press Done: next field"
              : "Press: finish digit");
}

void startLocationEdit() {
  double coords[] = {
    observerLat, observerLon
  };

  for (int i = 0; i < 2; i++) {
    magnitude[i] =
        lround(fabs(coords[i]) * 10000);

    negativeCoord[i] = coords[i] < 0;
  }

  editRow = 0;
  editMode = 0;
  editDigit = 3;
  page = LOCATION;
}

void saveLocation() {
  observerLat =
      magnitude[0] / 10000.0 *
      (negativeCoord[0] ? -1 : 1);

  observerLon =
      magnitude[1] / 10000.0 *
      (negativeCoord[1] ? -1 : 1);

  bool saved = false;

  if (prefsReady) {
    size_t a = prefs.putDouble("lat", observerLat);
    size_t b = prefs.putDouble("lon", observerLon);

    saved =
        a == sizeof(double) &&
        b == sizeof(double);
  }

  updateNearby();

  notice = saved
      ? "Location saved"
      : "Applied for this session";

  page = SETTINGS;
}

// ---------- Wi-Fi placeholder ----------

void drawWifi() {
  frame("Wi-Fi");

  detailField(
      61, "Connection setup", "Captive portal");

  textAt(17, 124, "We will add provisioning");
  textAt(17, 144, "in the next Wi-Fi step.");

  textAt(17, 183, "No Wi-Fi connection in");
  textAt(17, 203, "this display demo.");

  textAt(8, 311, "Press / hold: settings");
}

// ---------- Screen routing ----------

void drawScreen() {
  switch (page) {
    case RADAR:
      drawRadar();
      break;

    case DETAILS:
      drawDetails();
      break;

    case SETTINGS:
      drawSettings();
      break;

    case LOCATION:
      drawLocation();
      break;

    case WIFI_PAGE:
      drawWifi();
      break;
  }
}

// ---------- Rotation handling ----------

void handleTurn(int steps) {
  switch (page) {
    case RADAR:
      radarCursor = wrap(
          radarCursor + steps, visibleCount + 1);
      break;

    case DETAILS:
      detailsPage = wrap(detailsPage + steps, 2);
      break;

    case SETTINGS:
      selectedSetting = wrap(
          selectedSetting + steps, 3);

      notice = "";
      break;

    case LOCATION:
      if (editMode == 0) {
        // Move between Latitude, Longitude, Save and Cancel.
        editRow = wrap(editRow + steps, 4);

      } else if (editMode == 1) {
        // Eight sign/digit positions plus Done.
        editDigit = wrap(editDigit + steps, 9);

      } else if (editDigit == 0) {
        if (steps % 2 != 0) {
          negativeCoord[editRow] =
              !negativeCoord[editRow];
        }

      } else if (editDigit < 8) {
        int64_t value =
            magnitude[editRow] +
            (int64_t)steps * digitPlace[editDigit];

        int32_t limit =
            editRow == 0 ? 900000 : 1800000;

        if (value < 0) value = 0;
        if (value > limit) value = limit;

        magnitude[editRow] = (int32_t)value;
      }
      break;

    case WIFI_PAGE:
      break;
  }

  redraw = true;
}

// ---------- Short press handling ----------

void handlePress() {
  switch (page) {
    case RADAR:
      if (radarCursor == visibleCount) {
        selectedSetting = 0;
        notice = "";
        page = SETTINGS;
      } else {
        selectedAircraft = visible[radarCursor];
        detailsPage = 0;
        page = DETAILS;
      }
      break;

    case DETAILS:
      page = RADAR;
      break;

    case SETTINGS:
      if (selectedSetting == 0) {
        startLocationEdit();
      } else if (selectedSetting == 1) {
        page = WIFI_PAGE;
      } else {
        page = RADAR;
      }
      break;

    case LOCATION:
      if (editMode == 2) {
        // Finish editing this digit.
        editMode = 1;

      } else if (editMode == 1) {
        if (editDigit == 8) {
          // Done advances to the next row.
          editMode = 0;
          editDigit = 3;

          // Latitude -> Longitude -> Save
          editRow++;
        } else {
          editMode = 2;
        }

      } else if (editRow < 2) {
        // Open Latitude or Longitude.
        editDigit = 3;
        editMode = 1;

      } else if (editRow == 2) {
        saveLocation();

      } else {
        // Cancel discards the draft coordinates.
        page = SETTINGS;
      }
      break;

    case WIFI_PAGE:
      page = SETTINGS;
      break;
  }

  redraw = true;
}

// ---------- Long press handling ----------

void handleHold() {
  if (page == LOCATION) {
    if (editMode > 0) {
      editMode = 0;
    } else {
      page = SETTINGS;
    }
  } else if (page == WIFI_PAGE) {
    page = SETTINGS;
  } else {
    page = RADAR;
  }

  redraw = true;
}

// ---------- Button debounce ----------

void readButton() {
  uint32_t now = millis();
  bool reading = digitalRead(ENC_SW);

  if (reading != lastReading) {
    lastReading = reading;
    changedAt = now;
  }

  if (now - changedAt >= 30 &&
      reading != buttonState) {
    buttonState = reading;

    if (buttonState == LOW) {
      pressedAt = now;
      holdHandled = false;
    } else if (!holdHandled) {
      handlePress();
    }
  }

  if (buttonState == LOW &&
      !holdHandled &&
      now - pressedAt >= 900) {
    holdHandled = true;
    handleHold();
  }
}

// ---------- Setup ----------

void setup() {
  Serial.begin(115200);

  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_SW, INPUT_PULLUP);

  SPI.begin(18, 19, 23, TFT_CS);

  tft.begin();
  tft.setRotation(0);  // Portrait: 240 x 320
  tft.setTextWrap(false);

  prefsReady =
      prefs.begin("radar-location", false);

  if (prefsReady) {
    double lat =
        prefs.getDouble("lat", DEMO_LAT);

    double lon =
        prefs.getDouble("lon", DEMO_LON);

    if (isfinite(lat) &&
        isfinite(lon) &&
        fabs(lat) <= 90 &&
        fabs(lon) <= 180) {
      observerLat = lat;
      observerLon = lon;
    }
  }

  seedDemoPositions();
  updateNearby();

  attachInterrupt(
      digitalPinToInterrupt(ENC_CLK),
      readEncoder,
      FALLING);
}

// ---------- Main loop ----------

void loop() {
  portENTER_CRITICAL(&encoderMux);

  int steps = encoderSteps;
  encoderSteps = 0;

  portEXIT_CRITICAL(&encoderMux);

  if (steps != 0) {
    handleTurn(steps);
  }

  readButton();

  if (redraw) {
    redraw = false;
    drawScreen();
  }

  delay(1);
}