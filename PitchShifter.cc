#ifndef PitchShifter_h_
#define PitchShfiter_h_

#include "AudioStream_F32.h"
#include <arm_math.h>
#include <Arduino.h>

#include "mathDSP_F32.h"

namespace zao {

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
      float32_t *_input_data;
      float32_t *_output_data;
      float32_t *_fft_buffer;
      float32_t *_window;
      int _latency;
      int _rover;
      audio_block_f32_t *_input[1];
      int _fft_size;
      int _fft_size_2;
      int _hop_size;
      int _osamp;
      int _tones_per_octave;
      int _shift_factor;
      float _sample_rate;
      int _audio_block_size;
      bool _is_rad4;

      arm_cfft_radix4_instance_f32 _fft_inst_r4;
      arm_cfft_radix2_instance_f32 _fft_inst_r2;

      arm_cfft_radix4_instance_f32 _ifft_inst_r4;
      arm_cfft_radix2_instance_f32 _ifft_inst_r2;

      float32_t *_last_phase;
      float32_t *_sum_phase;
      float32_t *_input_freq;
      float32_t *_input_mag;
      float32_t *_output_acc;
  };

  PitchShifter::PitchShifter(
    PitchShifterInit &init
  )
    : AudioStream_F32(1, _input),
    _input_data(new float32_t[init.fft_size]),
    _output_data(new float32_t[init.fft_size]),
    _fft_buffer(new float32_t[2 * init.fft_size]),
    _window(new float32_t[init.fft_size]),
    _latency(init.fft_size - init.hop_size),
    _rover(_latency),
    _fft_size(init.fft_size),
    _fft_size_2(init.fft_size / 2),
    _hop_size(init.hop_size),
    _osamp(round(init.fft_size / init.hop_size)),
    _tones_per_octave(init.tones_per_octave),
    _shift_factor(init.shift_factor),
    _sample_rate(init.sample_rate),
    _audio_block_size(init.audio_block_size),
    _is_rad4(
      _fft_size == 16 || _fft_size == 64 ||
      _fft_size == 256 || _fft_size == 1024
    ),
    _last_phase(new float32_t[_fft_size_2 + 1]),
    _sum_phase(new float32_t[_fft_size_2 + 1]),
    _input_freq(new float32_t[_fft_size_2 + 1]),
    _input_mag(new float32_t[_fft_size_2 + 1]),
    _output_acc(new float32_t[2 * _fft_size])
  {
    for (int i = 0; i < _fft_size; i++) {
      // _window[i] = (
      //   -.5 * cosf(2.0 * M_PI * (float32_t)i / (float32_t)_fft_size) + .5
      // );
      _window[i] = 0.5 * (1.0 - cosf(
        2.0 * M_PI * (float32_t)i / ((float32_t)(_fft_size - 1))
      ));
    }

    for (int i = 0; i < _fft_size; i++) {
      _input_data[i] = .0;
      _output_data[i] = .0;
    }

    for (int i = 0; i <= _fft_size_2; i++) {
      _last_phase[i] = .0;
      _sum_phase[i] = .0;
      _input_freq[i] = .0;
      _input_mag[i] = .0;
    }

    for (int i = 0; i < 2 * _fft_size; i++) {
      _output_acc[i] = .0;
      _fft_buffer[i] = .0;
    }

    if (_is_rad4) {
      arm_cfft_radix4_init_f32(&_fft_inst_r4, _fft_size, 0, 1);
      arm_cfft_radix4_init_f32(&_ifft_inst_r4, _fft_size, 1, 1);
    }

    if (!_is_rad4) {
      arm_cfft_radix2_init_f32(&_fft_inst_r2, _fft_size, 0, 1);
      arm_cfft_radix2_init_f32(&_ifft_inst_r2, _fft_size, 1, 1);
    }
  }

  PitchShifter::~PitchShifter() {
    if (_input_data != NULL) {
      delete _input_data;
    }

    if (_output_data != NULL) {
      delete _output_data;
    }

    if (_fft_buffer != NULL) {
      delete _fft_buffer;
    }

    if (_window != NULL) {
      delete _window;
    }

    if (_last_phase != NULL) {
      delete _last_phase;
    }

    if (_sum_phase != NULL) {
      delete _sum_phase;
    }

    if (_input_freq != NULL) {
      delete _input_freq;
    }

    if (_input_mag != NULL) {
      delete _input_mag;
    }
  }

  int PitchShifter::getTonesPerOctave() {
    return _tones_per_octave;
  }

  void PitchShifter::setTonesPerOctave(int tones_per_octave) {
    _tones_per_octave = tones_per_octave;
  }

  int PitchShifter::getShiftFactor() {
    return _shift_factor;
  }

  void PitchShifter::setShiftFactor(int shift_factor) {
    _shift_factor = shift_factor;
  }

  bool PitchShifter::enable(bool state = true) {
    _enabled = state;

    Serial.print("\t_latency=");
    Serial.println(_latency);

    return state;
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

    float32_t expected_phase_diff = (
      2.0 * M_PI * (float32_t)_hop_size / (float32_t)_fft_size
    );
    float32_t freq_per_bin = (float32_t)_sample_rate / (float32_t)_fft_size;

    float32_t min_v = 1.0;
    float32_t max_v = -1.0;

    for (int i = 0; i < _audio_block_size; i++) {
      _input_data[_rover] = in_out_audio_block->data[i];
      in_out_audio_block->data[i] = _output_data[_rover - _latency];
      _rover++;

      // We don't have enough samples to process
      if (_rover < _fft_size) {
        continue;
      }

      // We do have enough samples to process
      _rover = _latency;

      // window and interleave
      for (int k = 0; k < _fft_size; k++) {
        _fft_buffer[2 * k] = _input_data[k] * _window[k];
        _fft_buffer[2 * k + 1] = 0.;
      }

      // do analysis
      // do fft transform
      if (_is_rad4) {
        arm_cfft_radix4_f32(&_fft_inst_r4, _fft_buffer);
      } else {
        arm_cfft_radix2_f32(&_fft_inst_r2, _fft_buffer);
      }

      float32_t real, imag, mag, phase, tmp;
      float32_t *syn_mag = new float32_t[_fft_size];
      float32_t *syn_freq = new float32_t[_fft_size];

      for (int k = 0; k <= _fft_size; k++) {
        syn_mag[k] = 0;
        syn_freq[k] = 0;
      }

      // compute mag and phase
      for (int k = 0; k <= _fft_size_2; k++) {
        real = _fft_buffer[2 * k];
        imag = _fft_buffer[2 * k + 1];

        mag = (float32_t)2.0 * sqrtf(real * real + imag * imag);
        phase = atan2f(imag, real);
        tmp = phase - _last_phase[k];
        _last_phase[k] = phase;

        tmp -= k * expected_phase_diff;

        // qpd = tmp / M_PI;
        // if (qpd >= 0) qpd += qpd&1;
        // else qpd -= qpd&1;
        // tmp -= M_PI * (float32_t)qpd;

        tmp = fmod(tmp + M_PI, 2.0 * M_PI) - M_PI;

        // get freq deviation
        tmp = _osamp * tmp / (2.0 * M_PI);

        tmp = k * freq_per_bin + tmp * freq_per_bin;
        _input_mag[k] = mag;
        _input_freq[k] = tmp;

      }

      // processing (shifting the pitch)
      float32_t pitch_shift = pow(
        2,
        _shift_factor / _tones_per_octave
      );

      for (int k = 0; k <= _fft_size_2; k++) {
        int index = round(k * pitch_shift);
        if (index <= _fft_size_2) {
          syn_mag[index] += _input_mag[k];
          syn_freq[index] = _input_freq[k] * pitch_shift;
        }
      }

      // synthesis
      for (int k = 0; k <= _fft_size_2; k++) {
        mag = syn_mag[k];
        tmp = syn_freq[k];

        tmp -= (float32_t)k * freq_per_bin;
        tmp /= freq_per_bin;
        tmp = 2.0 * M_PI * tmp / _osamp;
        tmp += (float32_t)k * expected_phase_diff;

        _sum_phase[k] += tmp;
        phase = _sum_phase[k];

        _fft_buffer[2 * k] = mag * cosf(phase);
        _fft_buffer[2 * k + 1] = mag * sinf(phase);
      }

      // zero out negative freqs
      for (int k = _fft_size/* + 2 */; k < 2 * _fft_size; k++) {
        _fft_buffer[k] = 0.;
      }

      // ifft
      if (_is_rad4) {
        arm_cfft_radix4_f32(&_ifft_inst_r4, _fft_buffer);
      } else {
        arm_cfft_radix2_f32(&_ifft_inst_r2, _fft_buffer);
      }

      // window and aggregate
      for (int k = 0; k < _fft_size; k++) {
        // double win = -.5*cos(2.*M_PI*(double)k/(double)_fft_size)+.5;
        _output_acc[k] += (
          2.0 * _window[k] * _fft_buffer[2 * k]
          /* / (_fft_size_2  *  _osamp) */
        );
        // _output_acc[k] += _fft_buffer[2 * k] * _hop_size * _window[k];
      }

      // shift input and output buffers
      for (int k = 0; k < _hop_size; k++) {
        _output_data[k] = _output_acc[k];
      }

      memmove(
        _output_acc,
        _output_acc + _hop_size,
        _fft_size * sizeof(float32_t)
      );

      for (int k = 0; k < _latency; k++) {
        _input_data[k] = _input_data[k + _hop_size];
      }

      // _samples_available += _fft_size;

      delete syn_mag;
      delete syn_freq;
    }

    // if (_samples_available == 0) {
    //   AudioStream_F32::release(in_out_audio_block);
    //   return;
    // }

    // for (int i = 0; i < _audio_block_size; i++) {
    //   in_out_audio_block->data[i] = _output_data[i];
    // }


    // int n = min(_samples_available, _audio_block_size);
    // for (int i = 0; i < n; i++) {
    //   _samples_available--;
    //   in_out_audio_block->data[i] = _output_data[i];
    //   _output_data[i] = _output_data[i + _audio_block_size];
    // }

    AudioStream_F32::transmit(in_out_audio_block);
    AudioStream_F32::release(in_out_audio_block);
  }
}

#endif
