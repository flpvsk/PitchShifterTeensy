#ifndef PitchShifter_h_
#define PitchShfiter_h_

#include "AudioStream_F32.h"
#include "PitchShifterAlg.h"
#include <arm_math.h>

#include "DebugUtils.h"

// #include "mathDSP_F32.h"

void smbFft(float32_t *fftBuffer, long fftFrameSize, long sign)
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

namespace zao {

  class PitchShifterArmMath final: public PitchShifterMath<float32_t> {
    public:
      PitchShifterArmMath(int fft_size):
        _is_rad4(
          fft_size == 16 ||
          fft_size == 64 ||
          fft_size == 256 ||
          fft_size == 1024
        ),
        _fft_size(fft_size)
      {
        if (_is_rad4) {
          arm_cfft_radix4_init_f32(&_fft_inst_r4, _fft_size, 0, 1);
          arm_cfft_radix4_init_f32(&_ifft_inst_r4, _fft_size, 1, 1);
        }

        if (!_is_rad4) {
          arm_cfft_radix2_init_f32(&_fft_inst_r2, _fft_size, 0, 1);
          arm_cfft_radix2_init_f32(&_ifft_inst_r2, _fft_size, 1, 1);
        }
      }

      void fft(float32_t *fft_buffer, int fft_size) {
        smbFft(fft_buffer, fft_size, -1);
        return;

        if (fft_size != _fft_size) {
          log("[WARN] PitchShifterArmMath.fft() wrong fft_size");
          return;
        }

        if (_is_rad4) {
          arm_cfft_radix4_f32(&_fft_inst_r4, fft_buffer);
        } else {
          arm_cfft_radix2_f32(&_fft_inst_r2, fft_buffer);
        }
      }

      void ifft(float32_t *fft_buffer, int fft_size) {
        smbFft(fft_buffer, fft_size, 1);
        return;
        if (fft_size != _fft_size) {
          log("[WARN] PitchShifterArmMath.ifft() wrong fft_size");
          return;
        }

        if (_is_rad4) {
          arm_cfft_radix4_f32(&_ifft_inst_r4, fft_buffer);
        } else {
          arm_cfft_radix2_f32(&_ifft_inst_r2, fft_buffer);
        }
      }

      ~PitchShifterArmMath() {

      }

    private:
      const bool _is_rad4;
      const int _fft_size;

      arm_cfft_radix4_instance_f32 _fft_inst_r4;
      arm_cfft_radix2_instance_f32 _fft_inst_r2;

      arm_cfft_radix4_instance_f32 _ifft_inst_r4;
      arm_cfft_radix2_instance_f32 _ifft_inst_r2;
  };

  struct PitchShifterInit {
    int fft_size = 1024;
    int hop_size = 256;
    int tones_per_octave = 12;
    int shift_factor = 12;
    float sample_rate = AUDIO_SAMPLE_RATE;
    int audio_block_size = AUDIO_BLOCK_SAMPLES;
  };

  class PitchShifter: public AudioStream_F32 {

    public:
      PitchShifter(
        PitchShifterInit &init
      );

      ~PitchShifter();

      int getTonesPerOctave();
      void setTonesPerOctave(int tones_per_octave);
      int getShiftFactor();
      void setShiftFactor(int shift_factor);

      bool setEnabled(bool state);

      void update();

    private:
      bool _enabled = false;
      audio_block_f32_t *_input[1];
      int _audio_block_size;
      PitchShifterAlg<float32_t> *_alg;
      PitchShifterArmMath *_math;
  };

  PitchShifter::PitchShifter(PitchShifterInit &init) :
    AudioStream_F32(1, _input),
    _audio_block_size(init.audio_block_size)
  {

    _math = new PitchShifterArmMath(init.fft_size);

    PitchShifterAlgInit alg_init;
    alg_init.sample_rate = init.sample_rate;
    alg_init.shift_factor = init.shift_factor;
    alg_init.tones_per_octave = init.tones_per_octave;
    alg_init.fft_size = init.fft_size;
    alg_init.hop_size = init.hop_size;
    alg_init.audio_block_size = init.audio_block_size;

    _alg = new PitchShifterAlg<float32_t>(
      alg_init,
      _math
    );
  }

  PitchShifter::~PitchShifter() {
    delete _alg;
    delete _math;
  }

  int PitchShifter::getTonesPerOctave() {
    return _alg->getTonesPerOctave();
  }

  void PitchShifter::setTonesPerOctave(int tones_per_octave) {
    return _alg->setTonesPerOctave(tones_per_octave);
  }

  int PitchShifter::getShiftFactor() {
    return _alg->getShiftFactor();
  }

  void PitchShifter::setShiftFactor(int shift_factor) {
    _alg->setShiftFactor(shift_factor);
  }

  bool PitchShifter::setEnabled(bool state = true) {
    _enabled = state;
    return _enabled;
  }

  void PitchShifter::update() {
    audio_block_f32_t *in_out_audio_block =
      AudioStream_F32::receiveWritable_f32();

    if (!in_out_audio_block) {
      return;
    }

    if (!_enabled) {
      AudioStream_F32::transmit(in_out_audio_block);
      AudioStream_F32::release(in_out_audio_block);
      return;
    }

    float32_t in[_audio_block_size];
    float32_t out[_audio_block_size];

    for (int i = 0; i < _audio_block_size; i++) {
      in[i] = in_out_audio_block->data[i];
    }

    _alg->process(
      in,
      out
    );

    for (int i = 0; i < _audio_block_size; i++) {
      in_out_audio_block->data[i] = out[i];
    }

    AudioStream_F32::transmit(in_out_audio_block);
    AudioStream_F32::release(in_out_audio_block);
  }
}

#endif
