#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h> // Serial port configuration
#include <math.h>
#include <sys/mman.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <pthread.h>
#include "PmodNAV.h"
#include "PmodOLED.h"
#include "PmodGPS.h"

// Offsets for AXI GPIO Interrupt Controller (standard Xilinx AXI GPIO)
#define AXI_GPIO_GIER_OFFSET    0x11C // Global Interrupt Enable Register
#define AXI_GPIO_IER_OFFSET     0x128 // IP Interrupt Enable Register
#define GIER_ENABLE_MASK        0x80000000
#define IER_CHANNEL1_MASK       0x1

// LED GPIO Register constants and register offsets
#define IN 0
#define OUT 1
#define GPIO_MAP_SIZE 0x10000
#define GPIO_DATA_OFFSET 0x00
#define GPIO_TRI_OFFSET 0x04

// UDP Socket Configuration
#define MY_PORT 5000
#define LAPTOP_IP "192.168.1.100"
#define LAPTOP_PORT 5000

// Globals
PmodNAV NAVInst;
PmodOLED OLEDInst;
PmodGPS GPSInst;
static float fahren;
static char dataOut[50];
int nav_fd, oled_fd, led_fd, gps_fd;

// Struct and mutex for data handling across threads
typedef struct {
    // IMU data
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    // Barometer data
    float pressure;
    float tempF;
    // GPS Data
    double latitude;
    double longitude;
    int gpsFix; // If the GPS data is accurate and clear enough
    // Watchdog timer for when warning packets are recieved
    int warningTimer;
    // Timer to determine whether Zybo connected or not
    int connectionHeartbeat;
    // Address to write values to LEDs
    uint32_t* ledPtr;
    // 0 = not connected, 1 = connected to network
    int connected;
} TelemetryData;

// Shared data struct instance and mutex
TelemetryData sharedData;
pthread_mutex_t dataMutex = PTHREAD_MUTEX_INITIALIZER;

// Function prototypes
void enableNAVInterrupts(void *gpio_base_ptr);
double convertNMEAToDecimal(double nmea_val, char quadrant);
void celsToFah(float tempC);
void* mapUIOSubdevice(int uio_fd, int map_index, size_t size);
void* sensorThreadFunc(void* arg);
void* displayThreadFunc(void* arg);
void* networkThreadFunc(void* arg);
void* ledThreadFunc(void* arg);
void* gpsThreadFunc(void* arg);
void* webThreadFunc(void* arg);

int main() {
    void *nav_gpio_ptr, *nav_spi_ptr, *oled_gpio_ptr, *oled_spi_ptr, *led_ptr;
    volatile uint32_t *led_tri_reg;
    pthread_t sensor_tid, display_tid, networking_tid, led_tid, web_tid, gps_tid;
    // Open Devices
    nav_fd = open("/dev/uio1", O_RDWR);
    oled_fd = open("/dev/uio2", O_RDWR);
    led_fd = open("/dev/uio3", O_RDWR);
    gps_fd = open("/dev/uio0", O_RDWR); // Make it non-blocking and not able to control the end of program

    if (nav_fd < 0) {
        perror("Error opening Pmod NAV (/dev/uio1)");
        return -1;
    }

    if (oled_fd < 0) {
        perror("Error opening Pmod OLED (/dev/uio2)");
        return -1;
    }

    if (led_fd < 0) {
        perror("Failed to open LED GPIO block (/dev/uio3)");
        return -1;
    }

    if (gps_fd < 0) {
        perror("Failed to open Pmod GPS (/dev/uio0)");
        return -1;
    }

    // Map Hardware (Treat GPS as a terminal instead of a memory mapped device)
    printf("Mapping NAV GPIO...\n");
    nav_gpio_ptr = mapUIOSubdevice(nav_fd, 0, 0x1000);

    printf("Mapping NAV xSPI...\n");
    nav_spi_ptr  = mapUIOSubdevice(nav_fd, 1, 0x10000);

    printf("Mapping OLED GPIO...\n");
    oled_gpio_ptr = mapUIOSubdevice(oled_fd, 0, 0x1000);

    printf("Mapping OLED xSPI...\n");
    oled_spi_ptr  = mapUIOSubdevice(oled_fd, 1, 0x10000);

    printf("Mapping LEDs...\n");
    led_ptr = mapUIOSubdevice(led_fd, 0, 0x1000);

    printf("Mapping GPS...\n");
    // Map 0 is the GPIO (Reset pins)
    void* gps_gpio_ptr = mapUIOSubdevice(gps_fd, 0, 0x1000);

    // Map 1 is the UART (Data)
    void* gps_uart_ptr = mapUIOSubdevice(gps_fd, 1, 0x2000);

    if (gps_gpio_ptr == MAP_FAILED || gps_uart_ptr == MAP_FAILED) {
        perror("GPS Mapping Failed");
        return -1;
    }

    printf("Initializing NAV Driver...\n");
    NAV_begin(&NAVInst, (uintptr_t)nav_gpio_ptr, (uintptr_t)nav_spi_ptr);
    NAV_Init(&NAVInst);
    enableNAVInterrupts(nav_gpio_ptr); // Pass the GPIO pointer

    printf("Initializing OLED Driver...\n");
    OLED_Begin(&OLEDInst, (uintptr_t)oled_gpio_ptr, (uintptr_t)oled_spi_ptr, 0x0, 0x0);

    printf("Initializing GPS Driver...\n");
    // Initialize using the modified Library (passing virtual addresses)
    GPS_begin(&GPSInst, (uintptr_t)gps_gpio_ptr, (uintptr_t)gps_uart_ptr, 100000000);
    GPS_changeBaud(&GPSInst, 9600);

    printf("Initializaing LEDs...\n");
    // Create register-access pointers
    led_tri_reg = (volatile uint32_t *)((char*)led_ptr + GPIO_TRI_OFFSET);
    // Set all 32 bits to output (write 0)
    *led_tri_reg = 0x0;
    // Assign LED pointer
    pthread_mutex_lock(&dataMutex);
    sharedData.ledPtr = led_ptr;
    pthread_mutex_unlock(&dataMutex);

    printf("Initialization Complete. Starting V2X System...\n");

    // Start Threads
    pthread_create(&sensor_tid, NULL, sensorThreadFunc, NULL);
    pthread_create(&display_tid, NULL, displayThreadFunc, NULL);
    pthread_create(&networking_tid, NULL, networkThreadFunc, NULL);
    pthread_create(&led_tid, NULL, ledThreadFunc, NULL);
    pthread_create(&web_tid, NULL, webThreadFunc, NULL);
    pthread_create(&gps_tid, NULL, gpsThreadFunc, NULL);

    // Wait until threads are finished before cleanup (never occurs)
    pthread_join(sensor_tid, NULL);
    pthread_join(display_tid, NULL);
    pthread_join(networking_tid, NULL);
    pthread_join(led_tid, NULL);
    pthread_join(web_tid, NULL);
    pthread_join(gps_tid, NULL);

    // Cleanup
    munmap(nav_gpio_ptr, 0x1000);
    munmap(nav_spi_ptr, 0x10000);
    munmap(oled_gpio_ptr, 0x1000);
    munmap(oled_spi_ptr, 0x10000);
    munmap(led_ptr, 0x1000);
    close(nav_fd);
    close(oled_fd);
    close(led_fd);
    close(gps_fd);

    return 0;
}

// Simple Celsius to Fahrenheit conversion function for temperature
void celsToFah(float tempC) {
    fahren = (tempC * 1.8) + 32.0;
}

// Convert NMEA (DDMM.MMMM) to Decimal Degrees (DD.DDDD) for Pmod GPS
double convertNMEAToDecimal(double nmea_val, char quadrant) {
    int degrees = (int)(nmea_val / 100);
    double minutes = nmea_val - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);

    if (quadrant == 'S' || quadrant == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

// Configures GPIO interrupts on the NAV to pulse on data ready
void enableNAVInterrupts(void *gpio_base_ptr) {
    volatile uint32_t *gpio_ptr = (volatile uint32_t *)gpio_base_ptr;

    printf("Enabling AXI GPIO Interrupts...\n");
    // Enable Global Interrupts (GIER)
    gpio_ptr[AXI_GPIO_GIER_OFFSET / 4] = GIER_ENABLE_MASK;
    // Enable Channel 1 Interrupts (IER)
    gpio_ptr[AXI_GPIO_IER_OFFSET / 4] = IER_CHANNEL1_MASK;
    printf("AXI GPIO Interrupts Enabled.\n");

    // Configure Sensors (The Source)

    // Accelerometer / Gyroscope Interrupts
    printf("Configuring Accel/Gyro Interrupts...\n");
    NAV_ConfigInt(&NAVInst, NAV_INT_PIN_1, NAV_ACL_MSK_XLIE_XL | NAV_GYRO_MSK_XLIE_G, NAV_PAR_INT_ACTIVEHIGH, NAV_PAR_INT_PUSHPULL);

    // Magnetometer Interrupts (LIS3MDL)
    printf("Configuring Magnetometer Interrupts...\n");
    // 0xE0 enables interrupts for X, Y, and Z axes
    NAV_ConfigIntMAG(&NAVInst, 0xE0, NAV_PAR_INT_ACTIVEHIGH, 0); // 0 = not latched

    // Altimeter Interrupts
    printf("Configuring Altimeter Interrupts...\n");
    // NAV_ALT_MSK_INT_DRDY enables Data Ready interrupt
    NAV_ConfigIntALT(&NAVInst, NAV_ALT_MSK_INT_DRDY, NAV_PAR_INT_ACTIVEHIGH, NAV_PAR_INT_PUSHPULL, 0, 1, 0, 0);
}

// Helper to map UIO devices with better error reporting
void* mapUIOSubdevice(int uio_fd, int map_index, size_t size) {
    // Calculate offset: N * page_size (Linux UIO standard)
    off_t offset = map_index * getpagesize();

    void* ptr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, uio_fd, offset);

    if (ptr == MAP_FAILED) {
        printf("MMAP Failed for FD %d, Map Index %d, Size 0x%lx\n",
               uio_fd, map_index, (unsigned long)size);
        perror("Reason");
        exit(1);
    }
    return ptr;
}

// Convert NMEA String (DDMM.MMMM) to Decimal Degrees (DD.DDDD)
double convertStringCoordToDecimal(char* coordStr, char dir) {
    if (!coordStr || strlen(coordStr) < 4) return 0.0;

    double raw = atof(coordStr);

    // NMEA format is DDDMM.MMMM or DDMM.MMMM
    // The last two digits before the decimal are minutes. Everything before is degrees.
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);

    double decimal = degrees + (minutes / 60.0);

    // Handle Southern and Western hemispheres
    if (dir == 'S' || dir == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

// Gathers updated sensor data through a weighted moving average filter from PmodNAV and stores in the shared data Structure
void* sensorThreadFunc(void* arg) {
    // Variable to store the interrupt count
    int irq_count;

    // Enable the interrupt in the UIO hardware (write 1 to enable)
    int reenable = 1;
    write(nav_fd, &reenable, sizeof(int));
    const float alpha = 0.5f;

    // Initialize history with 0
    // Accelerometer
    static float filt_accelX = 0.0f;
    static float filt_accelY = 0.0f;
    static float filt_accelZ = 0.0f;
    // Gyroscope
    static float filt_gyroX = 0.0f;
    static float filt_gyroY = 0.0f;
    static float filt_gyroZ = 0.0f;
    // Magnetometer
    static float filt_magX = 0.0f;
    static float filt_magY = 0.0f;
    static float filt_magZ = 0.0f;

    // Previous sample (for Rate of Change / Jerk calculation)
    float prev_accelX = 0.0f, prev_accelY = 0.0f;

    // THRESHOLDS
    // 1.0g = Gravity. Hard braking is usually around 0.4g to 0.8g.
    const float BRAKE_THRESHOLD = 0.4f;

    // Jerk is the difference between two samples. High value = Impact/Pothole.
    const float JERK_THRESHOLD = 1.2f;

    // Flag to seed the filter on the very first run
    int first_run = 1;

    while(1) {
        // BLOCKING WAIT
        int ret = read(nav_fd, &irq_count, sizeof(int));

        if (ret != sizeof(int)) {
            // If read fails or returns early, don't crash, just loop
            continue;
        }
        // Read Raw Data
        NAV_GetData(&NAVInst);
        celsToFah(NAVInst.tempC);

        // Update shared data Structure (lock mutex)
        pthread_mutex_lock(&dataMutex);

        // Apply Filter (Exponential Moving Average)
        if (first_run) {
            // Take in raw data on the first run through
            filt_accelX = NAVInst.acclData.X;
            filt_accelY = NAVInst.acclData.Y;
            filt_accelZ = NAVInst.acclData.Z;

            filt_gyroX = NAVInst.gyroData.X;
            filt_gyroY = NAVInst.gyroData.Y;
            filt_gyroZ = NAVInst.gyroData.Z;

            filt_magX = NAVInst.magData.X;
            filt_magY = NAVInst.magData.Y;
            filt_magZ = NAVInst.magData.Z;

            first_run = 0;
        } else {
            // Keep track of previous value
            prev_accelX = filt_accelX;
            prev_accelY = filt_accelY;
            // Filter Formula: New = (Alpha * Raw) + ((1 - Alpha) * Old)
            filt_accelX = (alpha * NAVInst.acclData.X) + ((1.0f - alpha) * filt_accelX);
            filt_accelY = (alpha * NAVInst.acclData.Y) + ((1.0f - alpha) * filt_accelY);
            filt_accelZ = (alpha * NAVInst.acclData.Z) + ((1.0f - alpha) * filt_accelZ);

            filt_gyroX = (alpha * NAVInst.gyroData.X) + ((1.0f - alpha) * filt_gyroX);
            filt_gyroY = (alpha * NAVInst.gyroData.Y) + ((1.0f - alpha) * filt_gyroY);
            filt_gyroZ = (alpha * NAVInst.gyroData.Z) + ((1.0f - alpha) * filt_gyroZ);

            filt_magX = (alpha * NAVInst.magData.X) + ((1.0f - alpha) * filt_magX);
            filt_magY = (alpha * NAVInst.magData.Y) + ((1.0f - alpha) * filt_magY);
            filt_magZ = (alpha * NAVInst.magData.Z) + ((1.0f - alpha) * filt_magZ);
        }

        // Store filtered values
        sharedData.accelX = filt_accelX;
        sharedData.accelY = filt_accelY;
        sharedData.accelZ = filt_accelZ;

        sharedData.gyroX = filt_gyroX;
        sharedData.gyroY = filt_gyroY;
        sharedData.gyroZ = filt_gyroZ;

        sharedData.magX = filt_magX;
        sharedData.magY = filt_magY;
        sharedData.magZ = filt_magZ;

        // Output unfiltered temperature and pressure (not as important)
        sharedData.pressure = NAVInst.hPa;
        sharedData.tempF = fahren;

        // Automatic warning detection
        int trigger = 0;
        // Measure on X and Y axis for high acceleration and jerk (Z-axis is always 1 due to gravity)
        // Calculate Jerk (Rate of change)
        float jerkX = fabsf(filt_accelX - prev_accelX);
        float jerkY = fabsf(filt_accelY - prev_accelY);
        // Condition A: Sustained Hard Braking (High G-force)
        if (fabsf(filt_accelY) > BRAKE_THRESHOLD || fabsf(filt_accelX) > BRAKE_THRESHOLD) {
            trigger = 1;
        }
        // Condition B: Sudden Impact (High Jerk)
        if (jerkX > JERK_THRESHOLD || jerkY > JERK_THRESHOLD) {
            trigger = 1;
        }
        // Activate warning timer when condition is met
        if (trigger) {
            sharedData.warningTimer = 25; // Trigger for 2.5 seconds (or keep refreshing)
        }
        pthread_mutex_unlock(&dataMutex);
        // End interrupt sequence, reenable to trigger on the next data byte available.
        write(nav_fd, &reenable, sizeof(int));
    }
}

void* displayThreadFunc(void* arg) {
    TelemetryData localData;
    int flashState = 0;
    int iteration = 0;

    // Buffer for formatting lines (16 chars + null terminator)
    char line_buf[20];

    while (1) {
        // Read Shared Data
        pthread_mutex_lock(&dataMutex);
        localData = sharedData;
        if (sharedData.warningTimer > 0) sharedData.warningTimer--;
        pthread_mutex_unlock(&dataMutex);
        // Determine if there is a warning
        if (localData.warningTimer > 0) {
            // Warning mode
            // Flash logic
            if (flashState) {
                // Flash text when warning triggered
                OLED_SetCursor(&OLEDInst, 0, 0);
                OLED_PutString(&OLEDInst, "                "); // Clear line
                OLED_SetCursor(&OLEDInst, 0, 1);
                OLED_PutString(&OLEDInst, "   WARNING!     ");
                OLED_SetCursor(&OLEDInst, 0, 2);
                OLED_PutString(&OLEDInst, "   BRAKE!!      ");
                OLED_SetCursor(&OLEDInst, 0, 3);
                OLED_PutString(&OLEDInst, "                "); // Clear line
                flashState = 0;
            } else {
                // Text Off (Blink effect)
                OLED_Clear(&OLEDInst);
                flashState = 1;
            }
        }
        else if (localData.connected) {
            // Reset the offline splash counter
            iteration = 0;

            OLED_SetCursor(&OLEDInst, 0, 0);
            OLED_PutString(&OLEDInst, "   PORT LIVE    ");
            OLED_SetCursor(&OLEDInst, 0, 1);
            OLED_PutString(&OLEDInst, "      AT        ");
            OLED_SetCursor(&OLEDInst, 0, 2);
            OLED_PutString(&OLEDInst, " 192.168.1.100  ");
            OLED_SetCursor(&OLEDInst, 0, 3);
            OLED_PutString(&OLEDInst, "     :8080      ");
            OLED_Update(&OLEDInst);
        }
        else {
            // Normal offline operation
            // Splash Screen Logic
            if (iteration == 0) {
                OLED_Clear(&OLEDInst);
                OLED_SetCursor(&OLEDInst, 0, 1);
                OLED_PutString(&OLEDInst, "   ENTERING     ");
                OLED_SetCursor(&OLEDInst, 0, 2);
                OLED_PutString(&OLEDInst, "    OFFLINE     ");
                OLED_Update(&OLEDInst);
                usleep(750000); // Hold for 0.75s
                OLED_Clear(&OLEDInst);
                iteration = 1; // Mark as shown
            }
            flashState = 0;
            // Accelerometer
            OLED_SetCursor(&OLEDInst, 0, 0);
            // Format: "A: 1.23  4.56   " (Pad with spaces to fill 16 chars)
            snprintf(line_buf, sizeof(line_buf), "A:%5.2f %5.2f  ", localData.accelX, localData.accelY);
            OLED_PutString(&OLEDInst, line_buf);

            // Gyroscope
            OLED_SetCursor(&OLEDInst, 0, 1);
            snprintf(line_buf, sizeof(line_buf), "G:%5.1f %5.1f  ", localData.gyroX, localData.gyroY);
            OLED_PutString(&OLEDInst, line_buf);

            // Magnetometer
            OLED_SetCursor(&OLEDInst, 0, 2);
            snprintf(line_buf, sizeof(line_buf), "M:%5.1f %5.1f  ", localData.magX, localData.magY);
            OLED_PutString(&OLEDInst, line_buf);

            // Environment
            OLED_SetCursor(&OLEDInst, 0, 3);
            snprintf(line_buf, sizeof(line_buf), "P:%4.0f T:%3.0f   ", localData.pressure, localData.tempF);
            OLED_PutString(&OLEDInst, line_buf);
            iteration++;
        }

        // Push to Screen
        OLED_Update(&OLEDInst);

        // Timing for 5Hz refresh
        usleep(200000);
    }
    return NULL;
}

void* networkThreadFunc(void* arg) {
    int sockfd;
    struct sockaddr_in myAddr, laptopAddr;
    char recv_buf[128];
    TelemetryData localData;
    int flags;
    const int WARNING_DURATION_TICKS = 25;
    const int TIMEOUT_TICKS = 20;

    printf("Network thread started.\n");

    // Create UDP Socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        return NULL;
    }

    // Make it Non-Blocking
    // This ensures recvfrom() returns immediately if no data is there
    flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    // Bind an address to the port
    memset(&myAddr, 0, sizeof(myAddr));
    myAddr.sin_family = AF_INET;
    myAddr.sin_addr.s_addr = INADDR_ANY; // Listen on all interfaces
    myAddr.sin_port = htons(MY_PORT);

    if (bind(sockfd, (const struct sockaddr *)&myAddr, sizeof(myAddr)) < 0) {
        perror("Bind failed");
        close(sockfd);
        return NULL;
    }

    // Setup sending destination (laptop)
    memset(&laptopAddr, 0, sizeof(laptopAddr));
    laptopAddr.sin_family = AF_INET;
    laptopAddr.sin_port = htons(LAPTOP_PORT);
    laptopAddr.sin_addr.s_addr = inet_addr(LAPTOP_IP);

    while (1) {
        // RECEIVING
        int n = recvfrom(sockfd, recv_buf, sizeof(recv_buf) - 1, 0, NULL, NULL);
        pthread_mutex_lock(&dataMutex);
        if (n > 0) {
            sharedData.connectionHeartbeat = TIMEOUT_TICKS;
            recv_buf[n] = '\0'; // Null-terminate string
            // Check for "BRAKE" command
            if (strstr(recv_buf, "BRAKE") != NULL) {
                printf("WARNING RECEIVED: BRAKE!\n");
                // Load the warning timer to tick down
                sharedData.warningTimer = WARNING_DURATION_TICKS;
            }
        }
        // Tick the connection timeout timer down
        if (sharedData.connectionHeartbeat > 0) {
            sharedData.connectionHeartbeat--;
            sharedData.connected = 1; // Connection still maintained
        }
        else {
            sharedData.connected = 0; // Indicate connection lost after no sends/receives for 2 seconds
        }
        // SENDING
        // Get latest data
        localData = sharedData;
        pthread_mutex_unlock(&dataMutex);

        // Send it to laptop
        sendto(sockfd, &localData, sizeof(TelemetryData), 0,
               (const struct sockaddr *)&laptopAddr, sizeof(laptopAddr));

        // Rate Limit (10Hz)
        usleep(100000);
    }

    // Close socket
    close(sockfd);
    return NULL;
}

// Simple state machine to alter the state of the LEDs depending on operating state
void* ledThreadFunc(void* arg){
    // State variables for patterns
    int patternState = 0;
    int flashState = 0;
    TelemetryData localData;
    volatile uint32_t *ledDataReg; 

    while(1){
        // Read Shared Data
        pthread_mutex_lock(&dataMutex);
        localData = sharedData;
        pthread_mutex_unlock(&dataMutex);
        // Set LED data register
        if (localData.ledPtr == NULL) {
            usleep(100000);
            continue;
        }
        ledDataReg = (volatile uint32_t *)((char*)localData.ledPtr + GPIO_DATA_OFFSET);
        // Warning mode (fast blink sequence)
        if (localData.warningTimer > 0) {
            if(flashState){
                *ledDataReg = 0xF; // All LEDs on
                flashState = 0;
            }
            else {
                *ledDataReg = 0x0; // All LEDs off
                flashState = 1;
            }
            usleep(100000);
        }
        // Online operation mode (rolling right to left)
        else if (localData.connected) {
            *ledDataReg = (1 << patternState);
            patternState++;
            if (patternState > 3) patternState = 0; // Reset LEDs after the 4th one is reached
            usleep(200000);
        }
        // Offline operation mode (slowly blink 1st LED)
        else {
            if (flashState) {
                *ledDataReg = 0x1; // LED 0 ON
                flashState = 0;
            } else {
                *ledDataReg = 0x0; // OFF
                flashState = 1;
            }
            usleep(500000);
        }
    }
}

void* gpsThreadFunc(void* arg) {
    printf("GPS Thread Started (Polled Mode - NO FIX FILTER).\n");

    double tempLat = 0.0;
    double tempLon = 0.0;

    char *latStr, *lonStr;
    char nsDir, ewDir;

    while (1) {
        // Poll for data
        GPS_getData(&GPSInst); //

        //Check if a full sentence was received
        if (GPSInst.ping) { 
            GPS_formatSentence(&GPSInst); 

            // Retrieve strings from the driver struct
            latStr = GPS_getLatitude(&GPSInst); 
            lonStr = GPS_getLongitude(&GPSInst); 

            // Retrieve direction characters (N/S, E/W)
            nsDir = GPSInst.GGAdata.NS; 
            ewDir = GPSInst.GGAdata.EW; 

            // Convert NMEA strings to Decimal Degrees
            tempLat = convertStringCoordToDecimal(latStr, nsDir);
            tempLon = convertStringCoordToDecimal(lonStr, ewDir);

            // Update Shared Data Safely
            pthread_mutex_lock(&dataMutex);
            // Update the 'gpsFix' flag just for the UI colors
            if (GPS_isFixed(&GPSInst)) { 
                sharedData.gpsFix = 1;
            } else {
                sharedData.gpsFix = 0;
            }
            sharedData.latitude = tempLat;
            sharedData.longitude = tempLon;
            pthread_mutex_unlock(&dataMutex);

            // Debug print to confirm data is flowing even without fix
            printf("GPS Raw: %s | Lat: %f Lon: %f\n", GPSInst.recv, tempLat, tempLon);

            // Reset the ping flag
            GPSInst.ping = 0; 
        }

        usleep(1000);
    }
    return NULL;
}

void* webThreadFunc(void* arg) {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    char recv_buf[1024];
    char response_buffer[4096];
    TelemetryData localData;

    // Socket Setup
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) { perror("Web socket failed"); exit(EXIT_FAILURE); }
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) { perror("setsockopt"); exit(EXIT_FAILURE); }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(8080);
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) { perror("Web bind failed"); exit(EXIT_FAILURE); }
    if (listen(server_fd, 3) < 0) { perror("listen"); exit(EXIT_FAILURE); }
    printf("Web Server started on Port 8080\n");

    while(1) {
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            perror("accept");
            continue;
        }

        memset(recv_buf, 0, 1024);
        read(new_socket, recv_buf, 1024);

        pthread_mutex_lock(&dataMutex);
        localData = sharedData;
        sharedData.connectionHeartbeat = 20;
        pthread_mutex_unlock(&dataMutex);

        if (strstr(recv_buf, "GET /data") != NULL) {
            char *json_header = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n";
            snprintf(response_buffer, sizeof(response_buffer),
                     "%s{\"warn\": %d, \"ax\": %.2f, \"ay\": %.2f, \"az\": %.2f, \"gx\": %.2f, \"gy\": %.2f, \"gz\": %.2f, \"mx\": %.2f, \"my\": %.2f, \"mz\": %.2f, \"tf\": %.1f, \"pr\": %.0f, \"lat\": %.6f, \"lon\": %.6f, \"fix\": %d}",
                     json_header, localData.warningTimer > 0 ? 1 : 0, localData.accelX, localData.accelY, localData.accelZ,
                     localData.gyroX, localData.gyroY, localData.gyroZ, localData.magX, localData.magY, localData.magZ,
                     localData.tempF, localData.pressure, localData.latitude, localData.longitude, localData.gpsFix
            );
            send(new_socket, response_buffer, strlen(response_buffer), 0);
        }
        else {
            char *html_header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";
            snprintf(response_buffer, sizeof(response_buffer),
                     "%s"
                     "<html><head>"
                     "<link rel='stylesheet' href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css' />"
                     "<script src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js'></script>"
                     "<style>"
                     "body{font-family:sans-serif; background-color:#ffffff; text-align:center; padding:20px; transition: background-color 0.2s;}"
                     ".sensor-box{border:1px solid #333; display:inline-block; width:150px; padding:15px; margin:10px; border-radius:10px; background:white; box-shadow: 2px 2px 5px #ccc; vertical-align:top;}"
                     ".alert{background:red; color:white; font-size:40px; font-weight:bold; display:none; padding:20px; animation:blink 0.5s infinite;}"
                     "@keyframes blink{50%%{opacity:0;}}"
                     "#map { height: 350px; width: 80%%; margin: 20px auto; border: 2px solid #333; border-radius: 10px; z-index: 1; }"
                     "#gps-status { font-weight: bold; color: orange; }"
                     "</style>"
                     "<script>"
                     "var map, marker;"
                     "var firstLock = false;"
                     "function initMap() {"
                     "   map = L.map('map').setView([0, 0], 2);"
                     "   L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {maxZoom: 19, attribution: 'OpenStreetMap'}).addTo(map);"
                     "   marker = L.marker([0, 0]).addTo(map);"
                     "}"
                     "function fetchTelemetry() {"
                     "  fetch('/data').then(response => response.json()).then(data => {"
                     "      if(data.warn == 1) { document.getElementById('warn-box').style.display = 'block'; document.body.style.backgroundColor = '#ffcccc'; }"
                     "      else { document.getElementById('warn-box').style.display = 'none'; document.body.style.backgroundColor = '#ffffff'; }"

                     // Standard Sensors
                     "      document.getElementById('ax').innerText = data.ax; document.getElementById('ay').innerText = data.ay; document.getElementById('az').innerText = data.az;"
                     "      document.getElementById('gx').innerText = data.gx; document.getElementById('gy').innerText = data.gy; document.getElementById('gz').innerText = data.gz;"
                     "      document.getElementById('mx').innerText = data.mx; document.getElementById('my').innerText = data.my; document.getElementById('mz').innerText = data.mz;"
                     "      document.getElementById('tf').innerText = data.tf; document.getElementById('pr').innerText = data.pr;"
                     "      document.getElementById('txt-lat').innerText = data.lat.toFixed(6);"
                     "      document.getElementById('txt-lon').innerText = data.lon.toFixed(6);"

                     // Map Logic
                     "      if(data.fix == 1) {"
                     "          var newLatLng = new L.LatLng(data.lat, data.lon);"
                     "          marker.setLatLng(newLatLng);"
                     "          if (!firstLock) { map.setView(newLatLng, 17); firstLock = true; }"
                     "          document.getElementById('gps-status').innerText = 'GPS LOCKED';"
                     "          document.getElementById('gps-status').style.color = 'green';"
                     "      } else {"
                     // Still show'Searching' on the status, but the raw numbers above will update
                     "          document.getElementById('gps-status').innerText = 'SEARCHING (Raw Data Shown)...';"
                     "          document.getElementById('gps-status').style.color = 'orange';"
                     "      }"
                     "  }).catch(err => console.log('Offline'));"
                     "}"
                     "window.onload = function() { initMap(); setInterval(fetchTelemetry, 100); };"
                     "</script>"
                     "</head><body>"
                     "<div id='warn-box' class='alert'> BRAKE! </div>"
                     "<h1>Zybo V2X Dashboard</h1>"
                     "<h3>GPS Status: <span id='gps-status'>Initializing...</span></h3>"
                     "<div id='map'></div>"
                     "<hr>"
                     // GPS Data Box
                     "<div class='sensor-box'><h3>GPS Data</h3>Lat: <span id='txt-lat'>0</span><br>Lon: <span id='txt-lon'>0</span></div>"
                     // Other Sensors
                     "<div class='sensor-box'><h3>Accelerometer</h3>X: <span id='ax'>0</span><br>Y: <span id='ay'>0</span><br>Z: <span id='az'>0</span></div>"
                     "<div class='sensor-box'><h3>Gyroscope</h3>X: <span id='gx'>0</span><br>Y: <span id='gy'>0</span><br>Z: <span id='gz'>0</span></div>"
                     "<div class='sensor-box'><h3>Magnetometer</h3>X: <span id='mx'>0</span><br>Y: <span id='my'>0</span><br>Z: <span id='mz'>0</span></div>"
                     "<div class='sensor-box'><h3>Env</h3>Temp: <span id='tf'>0</span> F<br>Press: <span id='pr'>0</span> hPa</div>"
                     "</body></html>",
                     html_header
            );
            send(new_socket, response_buffer, strlen(response_buffer), 0);
        }
        shutdown(new_socket, SHUT_RDWR);
        close(new_socket);
    }
    return NULL;
}