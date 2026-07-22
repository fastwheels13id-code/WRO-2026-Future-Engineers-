#include "esp_camera.h"
#include "Arduino.h"

//======================
// Pines AI Thinker
//======================


#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM      0

#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define LED_PIN 33

const int CAM_WIDTH = 400;
const int CAM_HEIGHT = 296;

const int PIXEL_STEP = 2;

//======================
// Conversión RGB → HSV
//======================

void rgbToHsv(uint8_t r,
              uint8_t g,
              uint8_t b,
              float &h,
              float &s,
              float &v)
{
    float rf = r / 255.0;
    float gf = g / 255.0;
    float bf = b / 255.0;

    float maxv = max(rf, max(gf, bf));
    float minv = min(rf, min(gf, bf));

    float delta = maxv - minv;

    v = maxv * 100.0;

    if (maxv == 0)
        s = 0;
    else
        s = delta / maxv * 100.0;

    if (delta == 0)
    {
        h = 0;
    }
    else
    {
        if (maxv == rf)
            h = 60 * fmod(((gf - bf) / delta), 6);

        else if (maxv == gf)
            h = 60 * (((bf - rf) / delta) + 2);

        else
            h = 60 * (((rf - gf) / delta) + 4);
    }

    if (h < 0)
        h += 360;
}

//======================
// Posición Horizontal
//======================

String obtenerPosicion(int centroX)
{
    int tercio = CAM_WIDTH / 3;

    if (centroX < tercio)
        return "IZQUIERDA";

    if (centroX > tercio * 2)
        return "DERECHA";

    return "CENTRO";
}

//====================================================
// Procesamiento de la imagen
//====================================================

void procesarImagen() {
    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb)
    {
        Serial.println("Error capturando imagen");
        return;
    }

    uint8_t *rgb_buf = NULL;

    size_t rgb_size = CAM_WIDTH * CAM_HEIGHT * 2;

    rgb_buf = (uint8_t *)malloc(rgb_size);

    if (rgb_buf == NULL)
    {
        esp_camera_fb_return(fb);
        return;
    }

    bool ok = jpg2rgb565(fb->buf,
                         fb->len,
                         rgb_buf,
                         JPG_SCALE_NONE);

    if (!ok)
    {
        free(rgb_buf);
        esp_camera_fb_return(fb);
        return;
    }

    uint16_t *buf = (uint16_t *)rgb_buf;

    long redPixels = 0;
    long greenPixels = 0;
    long magentaPixels = 0;
    long bluePixels = 0;
    long orangePixels = 0;

    long redSumX = 0;
    long redSumY = 0;

    long greenSumX = 0;
    long greenSumY = 0;

    long magentaSumX = 0;
    long magentaSumY = 0;

    long blueSumX = 0;
    long blueSumY = 0;

    long orangeSumX = 0;
    long orangeSumY = 0;

    float orangeHueSum = 0;
    float blueHueSum = 0;
    float redHueSum = 0;
    float greenHueSum = 0;
    float mangentaHueSum = 0;

    //------------------------------------------------
    // Escaneo
    //------------------------------------------------

    for (int y = 10; y < CAM_HEIGHT - 10; y += PIXEL_STEP)
    {
        for (int x = 10; x < CAM_WIDTH - 10; x += PIXEL_STEP)
        {
            int index = y * CAM_WIDTH + x;

            uint16_t pixel = buf[index];

            uint8_t r = ((pixel >> 11) & 0x1F) << 3;
            uint8_t g = ((pixel >> 5) & 0x3F) << 2;
            uint8_t b = (pixel & 0x1F) << 3;

            float h, s, v;

            rgbToHsv(r, g, b, h, s, v);

            if (x == CAM_WIDTH / 2 && y == CAM_HEIGHT / 2)
            {
                Serial.print(" Hue: ");
                Serial.println(h);
            }
            //------------------------------------------------
            // Ignorar colores apagados
            //------------------------------------------------

            // Ignorar colores poco saturados
            //if (s < 58)
              //  continue;

            // Ignorar zonas oscuras
            //if (v < 40)
              //  continue;

            //================ ROJO =================
            if ((h <= 10 || h >= 352) && s >=70 && v >=35 )
            {
                redPixels++;
                redSumX += x;
                redSumY += y;
            }

            //================ VERDE =================
            else if (h >= 95 && h <= 135 && s >= 60 && v >= 45)
            {
                greenPixels++;
                greenSumX += x;
                greenSumY += y;
            }

            //================ MAGENTA =================
            else if (h >= 305 && h <= 345 && s >= 35 && v >= 60)
            {
                magentaPixels++;
                magentaSumX += x;
                magentaSumY += y;
            }
            //================ AZUL =================
            else if (h >= 200 && h <= 245 &&  s >= 60 && v >= 40)
            {
                bluePixels++;
                blueSumX += x;
                blueSumY += y;
            }
            //================ NARANJA =================
            else if (h >= 15 && h <= 32 && s >= 65 && v >= 50)
            {
                orangePixels++;
                orangeSumX += x;
                orangeSumY += y;

                orangeHueSum += h;
            }
        }
    }

    //------------------------------------------------
    // Umbral mínimo
    //------------------------------------------------

    const int threshold = 130;

    if (redPixels > threshold && redPixels > greenPixels && redPixels > orangePixels && redPixels > magentaPixels && redPixels > bluePixels) 
    {
        int cx = redSumX / redPixels;
        int cy = redSumY / redPixels;

        String pos = obtenerPosicion(cx);

        Serial.println();
        Serial.println("==============================");
        Serial.println("OBJETO ROJO");
        Serial.print("Posicion : ");
        Serial.println(pos);
        Serial.print("Centro X : ");
        Serial.println(cx);
        Serial.print("Centro Y : ");
        Serial.println(cy);
        Serial.print("Pixeles  : ");
        Serial.println(redPixels);
        Serial.println("==============================");

        digitalWrite(LED_PIN, LOW);
    }

    else if (greenPixels > threshold && greenPixels > redPixels && greenPixels > bluePixels && greenPixels > magentaPixels && greenPixels > bluePixels && greenPixels > orangePixels)
    {
        int cx = greenSumX / greenPixels;
        int cy = greenSumY / greenPixels;

        String pos = obtenerPosicion(cx);

        Serial.println();
        Serial.println("==============================");
        Serial.println("OBJETO VERDE");
        Serial.print("Posicion : ");
        Serial.println(pos);
        Serial.print("Centro X : ");
        Serial.println(cx);
        Serial.print("Centro Y : ");
        Serial.println(cy);
        Serial.print("Pixeles  : ");
        Serial.println(greenPixels);
        Serial.println("==============================");

        digitalWrite(LED_PIN, HIGH);
    }
    else if (magentaPixels > threshold && magentaPixels > redPixels && magentaPixels > greenPixels && magentaPixels > orangePixels && magentaPixels > bluePixels)
    {
        int cx = magentaSumX / magentaPixels;
        int cy = magentaSumY / magentaPixels;

        String pos = obtenerPosicion(cx);

        Serial.println();
        Serial.println("==============================");
        Serial.println("OBJETO MAGENTA");
        Serial.print("Posicion : ");
        Serial.println(pos);
        Serial.print("Centro X : ");
        Serial.println(cx);
        Serial.print("Centro Y : ");
        Serial.println(cy);
        Serial.print("Pixeles  : ");
        Serial.println(magentaPixels);
        Serial.println("==============================");
    }

    else if (orangePixels > threshold &&
            orangePixels > redPixels &&
            orangePixels > greenPixels &&
            orangePixels > magentaPixels &&
            orangePixels > bluePixels)
    {
        int cx = orangeSumX / orangePixels;
        int cy = orangeSumY / orangePixels;

        String pos = obtenerPosicion(cx);

        Serial.println();
        Serial.println("==============================");
        Serial.println("OBJETO NARANJA");
        Serial.print("Posicion : ");
        Serial.println(pos);
        Serial.print("Centro X : ");
        Serial.println(cx);
        Serial.print("Centro Y : ");
        Serial.println(cy);
        Serial.print("Pixeles  : ");
        Serial.print(orangePixels);
        Serial.print("Hue promedio: ");
        Serial.println(orangeHueSum / orangePixels);

        Serial.println("==============================");
    }

    else if (bluePixels > threshold &&
            bluePixels > redPixels &&
            bluePixels > greenPixels &&
            bluePixels > magentaPixels &&
            bluePixels > orangePixels)
    {
        int cx = blueSumX / bluePixels;
        int cy = blueSumY / bluePixels;

        String pos = obtenerPosicion(cx);

        Serial.println();
        Serial.println("==============================");
        Serial.println("OBJETO AZUL");
        Serial.print("Posicion : ");
        Serial.println(pos);
        Serial.print("Centro X : ");
        Serial.println(cx);
        Serial.print("Centro Y : ");
        Serial.println(cy);
        Serial.print("Pixeles  : ");
        Serial.println(bluePixels);
        Serial.println("==============================");
    }

    else
    {
        Serial.print("Rojo: ");
        Serial.print(redPixels);

        Serial.print("   Verde: ");
        Serial.print(greenPixels);

        Serial.print(" Magenta: ");
        Serial.print(magentaPixels);

        Serial.print(" Azul: ");
        Serial.print(bluePixels);

        Serial.print(" Naranja: ");
        Serial.print(orangePixels);
    }

    free(rgb_buf);

    esp_camera_fb_return(fb);
}

//==================================================
// Setup
//==================================================

void setup()
{
    Serial.begin(9600);

    pinMode(LED_PIN, OUTPUT);

    digitalWrite(LED_PIN, LOW);

    camera_config_t config;

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;

    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;

    config.pixel_format = PIXFORMAT_JPEG;

    config.frame_size = FRAMESIZE_CIF;

    config.jpeg_quality = 10;

    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);

    if (err != ESP_OK)
    {
        Serial.printf("Error inicializando cámara: 0x%x\n", err);

        while (true)
        {
            delay(1000);
        }
    }

    sensor_t *s = esp_camera_sensor_get();

    //----------------------------------------------------
    // Ajustes recomendados
    //----------------------------------------------------

    s->set_brightness(s, 0);

    s->set_contrast(s, 1);

    s->set_saturation(s, 2);

    s->set_special_effect(s, 0);

    s->set_whitebal(s, 1);

    s->set_wb_mode(s, 0);

    s->set_gain_ctrl(s, 1);

    s->set_exposure_ctrl(s, 1);

    s->set_gainceiling(s, GAINCEILING_8X);

    s->set_colorbar(s, 0);

    s->set_hmirror(s, 0);

    s->set_vflip(s, 0);

    Serial.println();
    Serial.println("-----------------------------------");
    Serial.println("ESP32-CAM Lista");
    Serial.println("Detección de color:");
    Serial.println("ROJO");
    Serial.println("VERDE");
    Serial.println("-----------------------------------");
}

//==================================================
// Loop
//==================================================

void loop(){
    procesarImagen();
    delay(10);
}