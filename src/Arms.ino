#include <driver/i2s.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

#define SAMPLE_RATE (20000)        // Tốc độ lấy mẫu
#define ADC_INPUT (ADC1_CHANNEL_0) // Chân GPIO36
#define I2S_DMA_BUF_LEN (400)      // Cấp 400 bytes RAM cho DMA
#define I2S_DMA_BUF_COUNT (16)     // Cấp 16 buffer
#define ZERO_POINT 2900            // Analog value
#define ZERO_POINT_VOLTAGE 2550    // mV

uint16_t buffer[I2S_DMA_BUF_LEN * I2S_DMA_BUF_COUNT] = {0};

void ADCDMAInit()
{
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = I2S_DMA_BUF_COUNT,
      .dma_buf_len = I2S_DMA_BUF_LEN,
      .use_apll = true,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};
  assert(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL) == ESP_OK);
  assert(i2s_set_adc_mode(ADC_UNIT_1, ADC_INPUT) == ESP_OK);
  assert(adc1_config_channel_atten(ADC_INPUT, ADC_ATTEN_DB_11) == ESP_OK);
  assert(i2s_adc_enable(I2S_NUM_0) == ESP_OK);
}

void setup()
{
  Serial.begin(115200);
  ADCDMAInit();
}

void loop()
{
  size_t bytes_read;
  uint32_t read_counter = 0;
  uint32_t averaged_reading = 0;
  uint64_t read_sum = 0;
  uint64_t rms_sum = 0;
  float vrms = 0;
  esp_adc_cal_characteristics_t chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &chars);
  while (1)
  {
    i2s_read(I2S_NUM_0, &buffer, sizeof(buffer), &bytes_read, 30);
    for (int i = 0; i < bytes_read / 2; ++i)
    {
      read_sum += (buffer[i] & 0x0FFF); // Chỉ lấy phần dữ liệu ADC
      // rms_sum += ((buffer[i] & 0x0FFF) - ZERO_POINT) * ((buffer[i] & 0x0FFF) - ZERO_POINT); // Analog
      uint32_t temp = (esp_adc_cal_raw_to_voltage(buffer[i] & 0x0FFF, &chars) - ZERO_POINT_VOLTAGE); // mV - Sử dụng thư viện esp_adc_cal tăng độ chính xác 
      rms_sum += temp * temp;
      ++read_counter;
    }
    if (read_counter == I2S_DMA_BUF_LEN * I2S_DMA_BUF_COUNT)
    {
      averaged_reading = read_sum / read_counter;
      uint32_t voltage = esp_adc_cal_raw_to_voltage(averaged_reading, &chars);
      Serial.printf("Averaged signal:%d, %d \n", voltage, averaged_reading); // Print compatible with Arduino Plotter
      rms_sum = rms_sum / read_counter;
      Serial.printf("SUM: %d\n", rms_sum);
      vrms = sqrt(rms_sum);
      Serial.printf("Vrms: %f\n", vrms);
      read_counter = 0;
      read_sum = 0;
    }
    delay(2000);
  }
}