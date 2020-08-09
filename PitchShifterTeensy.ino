#include "Audio.h"
#include "AudioStream_F32.h"
#include "AudioMixer_F32.h"
#include "OpenAudio_ArduinoLibrary.h"

#include "PitchShifter.h"

zao::PitchShifterInit pitch_shifter_init;
const float sample_rate_Hz = pitch_shifter_init.sample_rate;
const int audio_block_samples = pitch_shifter_init.audio_block_size;
AudioSettings_F32 audio_settings(sample_rate_Hz, audio_block_samples);

//create audio library objects for handling the audio
AudioInputI2S i2sIn;   // This I16 input/output is T4.x compatible
AudioConvert_I16toF32 cnvrt1;  // Convert to float
AudioMixer4_F32 mixer1;
zao::PitchShifter pitch_shifter(pitch_shifter_init);
AudioEffectGain_F32 gain1;
AudioConvert_F32toI16 cnvrt2;
AudioOutputI2S i2sOut;
AudioControlSGTL5000 codec;

//Make all of the audio connections
AudioConnection patchCord1(i2sIn, 0, cnvrt1, 0);

AudioConnection_F32 patchCord3(cnvrt1, 0, mixer1, 0);

AudioConnection_F32 patchCord2(cnvrt1, 0, pitch_shifter, 0);
AudioConnection_F32 patchCord4(pitch_shifter, 0, mixer1, 1);

AudioConnection_F32 patchCord6(mixer1, 0, cnvrt2, 0);
AudioConnection patchCord7(cnvrt2, 0, i2sOut, 0);

//control display and serial interaction
bool enable_printCPUandMemory = false;
void togglePrintMemoryAndCPU(void) {
  enable_printCPUandMemory = !enable_printCPUandMemory;
};

//inputs and levels
float input_gain_dB = 15.0f; //gain on the microphone
float vol_knob_gain_dB = 0.0; //will be overridden by volume knob

void setup() {
  Serial.begin(1);
  delay(1000);
  Serial.println("pitch shifter: starting setup()...");
  Serial.print("\t: sample rate (Hz) = ");
  Serial.println(audio_settings.sample_rate_Hz);
  Serial.print("\t: block size (samples) = ");
  Serial.println(audio_settings.audio_block_samples);

  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(10);
  AudioMemory_F32(40, audio_settings);

  codec.enable();
  codec.adcHighPassFilterEnable();
  codec.inputSelect(AUDIO_INPUT_LINEIN);

  mixer1.gain(0, 0.6);
  mixer1.gain(1, 0.6);
  pitch_shifter.setShiftFactor(0);
  pitch_shifter.setTonesPerOctave(19);
  pitch_shifter.setEnabled(true);
}

void loop() {
  printCPUandMemory(millis(), 3000);
}

void printCPUandMemory(
  unsigned long curTime_millis,
  unsigned long updatePeriod_millis
) {
  //static unsigned long updatePeriod_millis = 3000; //how many
  //milliseconds between updating gain reading?
  static unsigned long lastUpdate_millis = 0;

  //has enough time passed to update everything?
  if (curTime_millis < lastUpdate_millis) lastUpdate_millis = 0;
  //handle wrap-around of the clock
  if ((curTime_millis - lastUpdate_millis) > updatePeriod_millis) {
    //is it time to update the user interface?
    Serial.print("CPU Cur/Peak: ");
    Serial.print(audio_settings.processorUsage());
    Serial.print("%/");
    Serial.print(audio_settings.processorUsageMax());
    Serial.print("%,   ");
    Serial.print(" Dyn MEM Float32 Cur/Peak: ");
    Serial.print(AudioMemoryUsage_F32());
    Serial.print("/");
    Serial.print(AudioMemoryUsageMax_F32());
    Serial.print(" MEM Int16 Cur/Peak: ");
    Serial.print(AudioMemoryUsage());
    Serial.print("/");
    Serial.print(AudioMemoryUsageMax());
    Serial.println();

    lastUpdate_millis = curTime_millis;
  }
}
