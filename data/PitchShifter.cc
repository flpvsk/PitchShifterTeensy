#ifndef PitchShifter_h_
#define PitchShfiter_h_

#include "AudioStream_F32.h"
#include <arm_math.h>
#include <Arduino.h>
#include "PitchShifterAlg.cc"

#include <stdexcept>

#include "mathDSP_F32.h"

namespace zao {

  class PitchShifterArmMath: zao::PitchShifterMath<float32_t> {
    public:
      PitchShifterArmMath(int fft_size):
        _fft_size(fft_size),
        _is_rad4(fft_size % 4  == 0)
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
        if (fft_size != _fft_size) {
          throw std::invalid_argument(
            "fft_size should be the same as "
            "was used in the constructor"
          );
        }

        if (_is_rad4) {
          arm_cfft_radix4_f32(&_fft_inst_r4, _fft_buffer);
        } else {
          arm_cfft_radix2_f32(&_fft_inst_r2, _fft_buffer);
        }
      }

      void ifft(float32_t *fft_buffer, int fft_size) {
        if (fft_size != _fft_size) {
          throw std::invalid_argument(
            "fft_size should be the same as "
            "was used in the constructor"
          );
        }

        if (_is_rad4) {
          arm_cfft_radix4_f32(&_ifft_inst_r4, _fft_buffer);
        } else {
          arm_cfft_radix2_f32(&_ifft_inst_r2, _fft_buffer);
        }
      }

    private:
      const bool _is_rad4 = false;
      const int _fft_size;

      arm_cfft_radix4_instance_f32 _fft_inst_r4;
      arm_cfft_radix2_instance_f32 _fft_inst_r2;

      arm_cfft_radix4_instance_f32 _ifft_inst_r4;
      arm_cfft_radix2_instance_f32 _ifft_inst_r2;
  }

  struct PitchShifterInit {
    int fft_size = 1024;
    int hop_size = 128;
    int tones_per_octave = 12;
    int shift_factor = 6;
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

      bool enable(bool state);

      void update();

    private:
      bool _enabled = false;
      zao::PitchShifterAlg<float32_t> *_alg;
  };

  PitchShifter::PitchShifter(
    PitchShifterInit &init
  ) : AudioStream_F32(1, _input) {
    zao::PitchShifterArmMath math =
      new zao::PitchShifterArmMath(init.fft_size);

    zao::PitchShifterAlgInit<float> alg_init;
    alg_init.sample_rate = init.sample_rate;
    alg_init.shift_factor = init.shift_factor;
    alg_init.tones_per_octave = init.tones_per_octave;
    alg_init.fft_size = init.fft_size;
    alg_init.hop_size = init.hop_size;
    alg_init.audio_block_size = init.audio_block_size;

    _alg = new PitchShifterAlg<float32_t>(
      alg_init,
      math
    );
  }

  PitchShifter::~PitchShifter() {
    delete _alg;
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
    return _alg->setShiftFactor(shift_factor);
  }

  bool PitchShifter::enable(bool state = true) {
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

    _alg.process(
      in_out_audio_block->data,
      in_out_audio_block->data
    );

    AudioStream_F32::transmit(in_out_audio_block);
    AudioStream_F32::release(in_out_audio_block);
  }
}

#endif
