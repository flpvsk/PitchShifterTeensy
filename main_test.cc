#include <string>
#include <iostream>
#include "PitchShifterAlg.cc"
#include <cmath>
#include "lib/AudioFile/AudioFile.h"

void smbFft(float *fftBuffer, long fftFrameSize, long sign)
/*
	FFT routine, (C)1996 S.M.Bernsee. Sign = -1 is FFT, 1 is iFFT (inverse)
	Fills fftBuffer[0...2*fftFrameSize-1] with the Fourier transform of the
	time domain data in fftBuffer[0...2*fftFrameSize-1]. The FFT array takes
	and returns the cosine and sine parts in an interleaved manner, ie.
	fftBuffer[0] = cosPart[0], fftBuffer[1] = sinPart[0], asf. fftFrameSize
	must be a power of 2. It expects a complex input signal (see footnote 2),
	ie. when working with 'common' audio signals our input signal has to be
	passed as {in[0],0.,in[1],0.,in[2],0.,...} asf. In that case, the transform
	of the frequencies of interest is in fftBuffer[0...fftFrameSize].
*/
{
  float wr, wi, arg, *p1, *p2, temp;
  float tr, ti, ur, ui, *p1r, *p1i, *p2r, *p2i;
  long i, bitm, j, le, le2, k;

  for (i = 2; i < 2 * fftFrameSize - 2; i += 2) {
    for (bitm = 2, j = 0; bitm < 2 * fftFrameSize; bitm <<= 1) {
      if (i & bitm) j++;
      j <<= 1;
    }
    if (i < j) {
      p1 = fftBuffer + i;
      p2 = fftBuffer + j;
      temp = *p1;
      *(p1++) = *p2;
      *(p2++) = temp;
      temp = *p1;
      *p1 = *p2;
      *p2 = temp;
    }
  }
  for (
    k = 0, le = 2;
    k < (long) (log(fftFrameSize) / log(2.) + .5); k++) {
    le <<= 1;
    le2 = le >> 1;
    ur = 1.0;
    ui = 0.0;
    arg = M_PI / (le2 >> 1);
    wr = cos(arg);
    wi = sign * sin(arg);
    for (j = 0; j < le2; j += 2) {
      p1r = fftBuffer + j;
      p1i = p1r + 1;
      p2r = p1r + le2;
      p2i = p2r + 1;
      for (i = j; i < 2 * fftFrameSize; i += le) {
        tr = *p2r * ur - *p2i * ui;
        ti = *p2r * ui + *p2i * ur;
        *p2r = *p1r - tr;
        *p2i = *p1i - ti;
        *p1r += tr;
        *p1i += ti;
        p1r += le;
        p1i += le;
        p2r += le;
        p2i += le;
      }
      tr = ur * wr - ui * wi;
      ui = ur * wi + ui * wr;
      ur = tr;
    }
  }
}


class TestMath : public zao::PitchShifterMath<float> {
public:
  float sinf(float v) override {
    return sin(v);
  }

  float cosf(float v) override {
    return cos(v);
  }

  float atan2f(float a, float b) override {
    return atan2(a, b);
  }

  float modf(float v, float base) override {
    return fmod(v, base);
  }

  float sqrtf(float v) override {
    return sqrt(v);
  }

  float powf(float base, float exp) override {
    return pow(base, exp);
  }

  float roundf(float v) override {
    return round(v);
  }

  void fft(float *fft_buffer, int fft_size) override {
    return smbFft(fft_buffer, fft_size, -1);
  }

  void ifft(float *fft_buffer, int fft_size) override {
    return smbFft(fft_buffer, fft_size, 1);
  }

  inline float pi() override {
    return M_PI;
  }
};

int main() {
  std::cout << "Hi\n";
  AudioFile<float> audio_file;
  audio_file.load(
    "/Users/flpvsk/projects/Arduino/PitchShifterTeensy/test_audio"
    "/sine-440.wav"
  );

  audio_file.printSummary();


  auto math = new TestMath();
  zao::PitchShifterAlgInit<float> init;
  init.sample_rate = audio_file.getSampleRate();
  init.shift_factor = 4;
  init.fft_size = 512;
  init.hop_size = 64;
  init.audio_block_size = 128;


  float pitch_shift = math->powf(
    2,
    (float) init.shift_factor / (float) init.tones_per_octave
  );

  std::cout << "pitch_shift=" << pitch_shift << "\n";

  auto pitch_shifter = new zao::PitchShifterAlg<float>(init, math);
  int num_samples = audio_file.getNumSamplesPerChannel();
  float in[init.audio_block_size];
  float out[init.audio_block_size];

  AudioFile<float>::AudioBuffer out_audio;
  out_audio.resize(1);
  out_audio[0].resize(num_samples);

  for (int i = 0; i < num_samples; i++) {
    int frame_n = floor((float)i / init.audio_block_size);
    int frame_s = frame_n * init.audio_block_size;
    int index = i - frame_s;

    if (index == 0 && i != 0) {
      pitch_shifter->process(in, out);
      for (int j = 0; j < init.audio_block_size; j++) {
        int result_index = j + frame_s;
        out_audio[0][result_index] = out[j];
//        out_audio[0][result_index] = (
//         out[j] -
//         audio_file.samples[0][result_index]
//        );
      }
    }

    if (index == 0) {
      memset(in, 0, init.audio_block_size * sizeof(float));
      memset(out, 0, init.audio_block_size * sizeof(float));
    }

    in[index] = audio_file.samples[0][i];
  }

  bool ok = audio_file.setAudioBuffer(out_audio);
  std::cout << "Buffer set ok? " << ok;

  audio_file.save(
    "/Users/flpvsk/projects/Arduino/PitchShifterTeensy/test_audio/out"
    "/sine-440-result.wav"
  );

  exit(0);
}

