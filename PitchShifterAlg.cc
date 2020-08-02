#include <string>
#include <iostream>
#include <cstddef>

namespace zao {
  template<class FLOAT>
  class PitchShifterMath {
  public:
    virtual FLOAT cosf(FLOAT v) = 0;

    virtual FLOAT sinf(FLOAT v) = 0;

    virtual FLOAT atan2f(FLOAT a, FLOAT b) = 0;

    virtual FLOAT modf(FLOAT v, FLOAT base) = 0;

    virtual FLOAT sqrtf(FLOAT v) = 0;

    virtual FLOAT powf(FLOAT base, FLOAT exp) = 0;

    virtual FLOAT roundf(FLOAT f) = 0;

    virtual void fft(FLOAT *fft_buffer, int fft_size) = 0;

    virtual void ifft(FLOAT *fft_buffer, int fft_size) = 0;

    virtual FLOAT pi() = 0;
  };


  template<class FLOAT>
  struct PitchShifterAlgInit {
    int fft_size = 512;
    int hop_size = 64;
    int tones_per_octave = 12;
    int shift_factor = 12;
    float sample_rate = 44100.;
    int audio_block_size = 128;
  };

  template<class FLOAT>
  class PitchShifterAlg {
  public:
    PitchShifterAlg(
      PitchShifterAlgInit<FLOAT> init,
      PitchShifterMath<FLOAT> *pitch_shifter_math
    );

    ~PitchShifterAlg();

    void process(FLOAT *in, FLOAT *out);

  private:
    PitchShifterMath<FLOAT> *_math;
    FLOAT *_input_data;
    FLOAT *_output_data;
    FLOAT *_fft_buffer;
    FLOAT *_window;
    FLOAT *_window_out;
    int _latency;
    int _rover;
    int _fft_size;
    int _fft_size_2;
    int _hop_size;
    FLOAT _osamp;
    int _tones_per_octave;
    int _shift_factor;
    float _sample_rate;
    int _audio_block_size;

    FLOAT *_last_phase;
    FLOAT *_sum_phase;
    FLOAT *_input_freq;
    FLOAT *_input_mag;
    FLOAT *_output_acc;
  };

  template<typename FLOAT>
  PitchShifterAlg<FLOAT>::PitchShifterAlg(
    PitchShifterAlgInit<FLOAT> init,
    PitchShifterMath<FLOAT> *pitch_shifter_math
  ): _math(pitch_shifter_math),
    _input_data(new FLOAT[init.fft_size]),
    _output_data(new FLOAT[init.fft_size]),
    _fft_buffer(new FLOAT[2 * init.fft_size]),
    _window(new FLOAT[init.fft_size]),
    _window_out(new FLOAT[init.fft_size]),
    _latency(init.fft_size - init.hop_size),
    _rover(_latency),
    _fft_size(init.fft_size),
    _fft_size_2(init.fft_size / 2),
    _hop_size(init.hop_size),
    _osamp((FLOAT) init.fft_size / (FLOAT) init.hop_size),
    _tones_per_octave(init.tones_per_octave),
    _shift_factor(init.shift_factor),
    _sample_rate(init.sample_rate),
    _audio_block_size(init.audio_block_size),
    _last_phase(new FLOAT[_fft_size_2 + 1]),
    _sum_phase(new FLOAT[_fft_size_2 + 1]),
    _input_freq(new FLOAT[_fft_size_2 + 1]),
    _input_mag(new FLOAT[_fft_size_2 + 1]),
    _output_acc(new FLOAT[2 * _fft_size]) {

    for (int i = 0; i < _fft_size; i++) {
      _window[i] = (
        -.5 * _math->cosf(2.0 * _math->pi() * (FLOAT) i /
          (FLOAT) _fft_size) + .5
      );

      _window_out[i] = (
        .5 * (1.0 - _math->cosf(
          2.0 * _math->pi() * (FLOAT) i / (FLOAT) _fft_size)
        )
      );

      _window_out[i] = (
        .355768 -
          .487396 * _math->cosf(
            2.0 * _math->pi() * (FLOAT) i / (FLOAT) _fft_size
          ) +
          .144232 * _math->cosf(
            4.0 * _math->pi() * (FLOAT) i / (FLOAT) _fft_size
          ) -
          .012604 * _math->cosf(
            6.0 * _math->pi() * (FLOAT) i / (FLOAT) _fft_size
          )
      );
      // _window[i] = 0.5 * (1.0 - _math->cosf(
      //  2.0 * _math->pi() * (FLOAT) i / ((FLOAT) (_fft_size - 1))
      // ));
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
  }

  template<typename FLOAT>
  void PitchShifterAlg<FLOAT>::process(FLOAT *in, FLOAT *out) {
    FLOAT expected_phase_diff = (
      2.0 * _math->pi() * (FLOAT) _hop_size / (FLOAT) _fft_size
    );
    FLOAT freq_per_bin = (FLOAT) _sample_rate / (FLOAT) _fft_size;

    FLOAT min_v = 1.0;
    FLOAT max_v = -1.0;

    for (int i = 0; i < _audio_block_size; i++) {
      _input_data[_rover] = in[i];
      out[i] = _output_data[_rover - _latency];
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
      _math->fft(_fft_buffer, _fft_size);

      FLOAT real, imag, mag, phase, tmp;
      auto *syn_mag = new FLOAT[_fft_size];
      auto *syn_freq = new FLOAT[_fft_size];

      for (int k = 0; k <= _fft_size; k++) {
        syn_mag[k] = 0;
        syn_freq[k] = 0;
      }

      // compute mag and phase
      for (int k = 0; k <= _fft_size_2; k++) {
        real = _fft_buffer[2 * k];
        imag = _fft_buffer[2 * k + 1];

        mag = (FLOAT) 2.0 * _math->sqrtf(real * real + imag * imag);
        phase = _math->atan2f(imag, real);
        tmp = phase - _last_phase[k];
        _last_phase[k] = phase;

        tmp -= k * expected_phase_diff;

        // qpd = tmp / _math.pi();
        // if (qpd >= 0) qpd += qpd&1;
        // else qpd -= qpd&1;
        // tmp -= _math.pi() * (FLOAT)qpd;

        tmp = _math->modf(
          tmp + _math->pi(),
          2.0 * _math->pi()
        ) - _math->pi();

        // get freq deviation
        tmp = _osamp * tmp / (2.0 * _math->pi());

        tmp = k * freq_per_bin + tmp * freq_per_bin;

        _input_mag[k] = mag;
        _input_freq[k] = tmp;
      }

      // processing (shifting the pitch)
      FLOAT pitch_shift = _math->powf(
        2,
        (FLOAT) _shift_factor / (FLOAT) _tones_per_octave
      );

      for (int k = 0; k <= _fft_size_2; k++) {
        int index = _math->roundf(k * pitch_shift);
        if (index <= _fft_size_2) {
          syn_mag[index] += _input_mag[k];
          syn_freq[index] = _input_freq[k] * pitch_shift;
        }
      }

      auto fft_buffer_syn = new FLOAT[2 * _fft_size];
      for (int k = 0; k < 2 * _fft_size; k++) {
        fft_buffer_syn[k] = 0.0;
      }

      // synthesis
      for (int k = 0; k <= _fft_size_2; k++) {
        mag = syn_mag[k];
        tmp = syn_freq[k];

        tmp -= (FLOAT) k * freq_per_bin;
        tmp /= freq_per_bin;
        tmp = 2.0 * _math->pi() * tmp * _hop_size / _fft_size;
        tmp += (FLOAT) k * expected_phase_diff;

        _sum_phase[k] += tmp;
        phase = _sum_phase[k];

        if (max_v < mag) {
          max_v = mag;
          min_v = phase;
        }

        _fft_buffer[2 * k] = mag * _math->cosf(phase);
        _fft_buffer[2 * k + 1] = mag * _math->sinf(phase);
      }

      std::cout << "max=" << max_v << " f=" << min_v << "\n";


      // zero out negative freqs
      for (int k = _fft_size + 2; k < 2 * _fft_size; k++) {
        _fft_buffer[k] = 0.;
      }

      _math->ifft(_fft_buffer, _fft_size);

      // window and aggregate
      for (int k = 0; k < _fft_size; k++) {
        // double win = -.5*cosf(2.*_math->pi()*(double)k/(double)
        // _fft_size)+.5;
//        _output_acc[k] += (
//          2.0 * _window[k] * _fft_buffer[2 * k]
//            / (_fft_size_2 * _osamp)
//        );
        _output_acc[k] += (
          4.0 * _window_out[k] * _fft_buffer[2 * k] *
            (FLOAT) _hop_size /
            (_fft_size * _fft_size)
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
        _fft_size * sizeof(FLOAT)
      );

      for (int k = 0; k < _latency; k++) {
        _input_data[k] = _input_data[k + _hop_size];
      }

      delete[] syn_mag;
      delete[] syn_freq;
    }
  }

  template<typename FLOAT>
  PitchShifterAlg<FLOAT>::~PitchShifterAlg() {
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
}
