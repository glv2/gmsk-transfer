/*
This file is part of gmsk-transfer, a program to send or receive data
by radio using the GMSK modulation.

Copyright 2021 Guillaume LE VAILLANT

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <complex.h>
#include <libhackrf/hackrf.h>
#include <liquid/liquid.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define TAU (2 * M_PI)
#define SAMPLES_PER_SYMBOL 2
#define CRC LIQUID_CRC_32
#define FEC0 LIQUID_FEC_HAMMING74
//#define FEC1 LIQUID_FEC_REP3
#define FEC1 LIQUID_FEC_NONE

#define HACKRF_CHECK(funcall) \
{ \
  int e = funcall; \
  if(e != 0) \
  { \
    fprintf(stderr, "Error: %s\n", hackrf_error_name(e)); \
    exit(-1); \
  } \
}

typedef enum
  {
    IO,
    HACKRF
  } radio_type_t;

typedef union
{
  hackrf_device *hackrf;
} radio_device_t;

typedef struct
{
  radio_type_t type;
  radio_device_t device;
  cbuffercf buffer;
  pthread_mutex_t buffer_mutex;
  unsigned long int frequency;
  unsigned long int center_frequency;
} radio_t;

FILE *file = NULL;
unsigned char stop = 0;
unsigned char verbose = 0;

FILE *dump = NULL;

void dump_samples(float complex *samples, unsigned int samples_size)
{
  fwrite(samples, sizeof(float complex), samples_size, dump);
}

void apply_gain(float complex *samples, unsigned int samples_size, float gain)
{
  unsigned int i;

  for(i = 0; i < samples_size; i++)
  {
    samples[i] *= gain;
  }
}

void cs8_to_cf32(char *sample_cs8, float complex *sample_cf32)
{
  *sample_cf32 = (sample_cs8[0] + I * sample_cs8[1]) / 128.0;
}

void cf32_to_cs8(float complex sample_cf32, char *sample_cs8)
{
  sample_cs8[0] = (char) (crealf(sample_cf32) * 128.0);
  sample_cs8[1] = (char) (cimagf(sample_cf32) * 128.0);
}

unsigned int read_data(unsigned char *payload, unsigned int payload_size)
{
  unsigned int n = fread(payload, 1, payload_size, file);
  return(n);
}

void write_data(unsigned char *payload, unsigned int payload_size)
{
  fwrite(payload, 1, payload_size, file);
}

void send_to_radio(radio_t *radio,
                   float complex *samples,
                   unsigned int samples_size)
{
  unsigned int size;
  unsigned int i;
  unsigned int n;

  if(dump)
  {
    dump_samples(samples, samples_size);
  }

  switch(radio->type)
  {
  case IO:
    fwrite(samples, sizeof(float complex), samples_size, stdout);
    break;

  case HACKRF:
    i = 0;
    size = samples_size;
    while(size > 0)
    {
      n = cbuffercf_space_available(radio->buffer);
      if(n > 0)
      {
        if(n > size)
        {
          n = size;
        }
        pthread_mutex_lock(&radio->buffer_mutex);
        cbuffercf_write(radio->buffer, &samples[i], n);
        pthread_mutex_unlock(&radio->buffer_mutex);
        i += n;
        size -= n;
      }
      else if(stop)
      {
        break;
      }
      else
      {
        usleep(1);
      }
    }
    break;
  }
}

unsigned int receive_from_radio(radio_t *radio,
                                float complex *samples,
                                unsigned int samples_size)
{
  unsigned int size = 0;
  float complex *buffer_samples;

  switch(radio->type)
  {
  case IO:
    size = fread(samples, sizeof(float complex), samples_size, stdin);
    break;

  case HACKRF:
    if(cbuffercf_size(radio->buffer) > 0)
    {
      pthread_mutex_lock(&radio->buffer_mutex);
      cbuffercf_read(radio->buffer, samples_size, &buffer_samples, &size);
      memcpy(samples, buffer_samples, size * sizeof(float complex));
      cbuffercf_release(radio->buffer, size);
      pthread_mutex_unlock(&radio->buffer_mutex);
    }
    else
    {
      usleep(1);
    }
    break;
  }
  return(size);
}

int hackrf_sample_block_requested(hackrf_transfer *transfer)
{
  radio_t *radio = (radio_t *) transfer->tx_ctx;
  unsigned int requested = transfer->valid_length / 2;
  unsigned int size;
  unsigned int i;
  float complex *samples;

  if(stop)
  {
    return(-1);
  }

  pthread_mutex_lock(&radio->buffer_mutex);
  cbuffercf_read(radio->buffer, requested, &samples, &size);
  if(size < requested)
  {
    if(verbose)
    {
      fprintf(stderr, "U");
    }
    bzero(&transfer->buffer[2 * size], 2 * (requested - size));
  }
  for(i = 0; i < size; i++)
  {
    cf32_to_cs8(samples[i], (char *) &transfer->buffer[2 * i]);
  }
  cbuffercf_release(radio->buffer, size);
  pthread_mutex_unlock(&radio->buffer_mutex);

  return(0);
}

int hackrf_sample_block_received(hackrf_transfer *transfer)
{
  radio_t *radio = (radio_t *) transfer->rx_ctx;
  unsigned int n = transfer->valid_length / 2;
  unsigned int size;
  unsigned int i;
  float complex sample;

  if(stop)
  {
    return(-1);
  }

  pthread_mutex_lock(&radio->buffer_mutex);
  size = cbuffercf_space_available(radio->buffer);
  if(n > size)
  {
    if(verbose)
    {
      fprintf(stderr, "O");
    }
    n = size;
  }
  for(i = 0; i < n; i++)
  {
    cs8_to_cf32((char *) &transfer->buffer[2 * i], &sample);
    cbuffercf_push(radio->buffer, sample);
  }
  pthread_mutex_unlock(&radio->buffer_mutex);

  return(0);
}

void send_frames(radio_t *radio, float sample_rate, unsigned int baud_rate)
{
  gmskframegen frame_generator = gmskframegen_create();
  float resampling_ratio = sample_rate / (baud_rate * SAMPLES_PER_SYMBOL);
  msresamp_crcf resampler = msresamp_crcf_create(resampling_ratio, 60);
  unsigned int delay = (unsigned int) ceilf(msresamp_crcf_get_delay(resampler));
  unsigned int header_size = 8;
  unsigned char header[header_size];
  unsigned int payload_size = 40;
  unsigned char payload[payload_size];
  unsigned int n;
  unsigned int frame_samples_size = (baud_rate * SAMPLES_PER_SYMBOL) / 20; /* 50 ms */
  int frame_complete;
  unsigned int samples_size = (unsigned int) ceilf((frame_samples_size + delay) * resampling_ratio);
  float frequency_offset = (float) radio->frequency - radio->center_frequency;
  float center_frequency = frequency_offset / sample_rate;
  float cutoff_frequency = (frequency_offset + (baud_rate * 2)) / sample_rate;
  iirfilt_crcf filter = iirfilt_crcf_create_prototype(LIQUID_IIRDES_BUTTER,
                                                      (frequency_offset == 0) ?
                                                      LIQUID_IIRDES_LOWPASS :
                                                      LIQUID_IIRDES_BANDPASS,
                                                      LIQUID_IIRDES_SOS,
                                                      1,
                                                      fabsf(cutoff_frequency),
                                                      fabsf(center_frequency),
                                                      1,
                                                      60);
  nco_crcf oscillator = nco_crcf_create(LIQUID_NCO);
  float complex *frame_samples = malloc(frame_samples_size * sizeof(float complex));
  float complex *samples = malloc(samples_size * sizeof(float complex));

  if((frame_samples == NULL) || (samples == NULL))
  {
    fprintf(stderr, "Error: Memory allocation failed\n");
    return;
  }

  nco_crcf_set_phase(oscillator, 0);
  nco_crcf_set_frequency(oscillator, TAU * center_frequency);

  gmskframegen_set_header_len(frame_generator, header_size);
  memcpy(header, "GMSKXFER", 8);

  while(1)
  {
    if(stop)
    {
      break;
    }
    n = read_data(payload, payload_size);
    if(n == 0)
    {
      break;
    }
    gmskframegen_assemble(frame_generator, header, payload, n, CRC, FEC0, FEC1);
    frame_complete = 0;
    n = 0;
    while(!frame_complete)
    {
      frame_complete = gmskframegen_write_samples(frame_generator, &frame_samples[n]);
      n += SAMPLES_PER_SYMBOL;
      if(frame_complete || (n + SAMPLES_PER_SYMBOL > frame_samples_size))
      {
        apply_gain(frame_samples, n, 0.75);
        msresamp_crcf_execute(resampler, frame_samples, n, samples, &n);
        if(frequency_offset != 0)
        {
          nco_crcf_mix_block_up(oscillator, samples, samples, n);
        }
        iirfilt_crcf_execute_block(filter, samples, n, samples);
        send_to_radio(radio, samples, n);
        n = 0;
      }
    }
  }

  for(n = 0; n < frame_samples_size; n++)
  {
    samples[n] = 0;
  }
  msresamp_crcf_execute(resampler, samples, frame_samples_size, samples, &n);
  if(frequency_offset != 0)
  {
    nco_crcf_mix_block_down(oscillator, samples, samples, n);
  }
  iirfilt_crcf_execute_block(filter, samples, n, samples);
  send_to_radio(radio, samples, n);

  if(radio->type == HACKRF)
  {
    /* Wait until the radio has sent all the samples */
    while(cbuffercf_size(radio->buffer) > 0)
    {
      if(stop)
      {
        break;
      }
      else
      {
        usleep(1);
      }
    }
  }

  free(samples);
  free(frame_samples);
  nco_crcf_destroy(oscillator);
  iirfilt_crcf_destroy(filter);
  msresamp_crcf_destroy(resampler);
  gmskframegen_destroy(frame_generator);
}

int frame_received(unsigned char *header,
                   int header_valid,
                   unsigned char *payload,
                   unsigned int payload_size,
                   int payload_valid,
                   framesyncstats_s stats,
                   void *user_data)
{
  if(verbose)
  {
    if(!header_valid)
    {
      fprintf(stderr, "H");
    }
    if(!payload_valid)
    {
      fprintf(stderr, "P");
    }
  }
  write_data(payload, payload_size);
  return(0);
}

void receive_frames(radio_t *radio, float sample_rate, unsigned int baud_rate)
{
  gmskframesync frame_synchronizer = gmskframesync_create(frame_received, NULL);
  float resampling_ratio = (baud_rate * SAMPLES_PER_SYMBOL) / sample_rate;
  msresamp_crcf resampler = msresamp_crcf_create(resampling_ratio, 60);
  unsigned int delay = (unsigned int) ceilf(msresamp_crcf_get_delay(resampler));
  unsigned int n;
  unsigned int frame_samples_size = (baud_rate * SAMPLES_PER_SYMBOL) / 20; /* 50 ms */
  unsigned int samples_size = (unsigned int) floorf(frame_samples_size / resampling_ratio) + delay;
  float frequency_offset = (float) radio->frequency - radio->center_frequency;
  nco_crcf oscillator = nco_crcf_create(LIQUID_NCO);
  float cutoff_frequency = (baud_rate * 2) / sample_rate;
  iirfilt_crcf low_pass = iirfilt_crcf_create_lowpass(2, cutoff_frequency);
  float complex *frame_samples = malloc((frame_samples_size + delay) * sizeof(float complex));
  float complex *samples = malloc(samples_size * sizeof(float complex));

  if((frame_samples == NULL) || (samples == NULL))
  {
    fprintf(stderr, "Error: Memory allocation failed\n");
    return;
  }

  nco_crcf_set_phase(oscillator, 0);
  nco_crcf_set_frequency(oscillator, TAU * (frequency_offset / sample_rate));

  while(1)
  {
    if(stop)
    {
      break;
    }

    n = receive_from_radio(radio, samples, samples_size);
    if((n == 0) && (radio->type == IO))
    {
      break;
    }
    if(frequency_offset != 0)
    {
      nco_crcf_mix_block_down(oscillator, samples, samples, n);
    }
    iirfilt_crcf_execute_block(low_pass, samples, n, samples);
    if(dump)
    {
      dump_samples(samples, n);
    }
    msresamp_crcf_execute(resampler, samples, n, frame_samples, &n);
    gmskframesync_execute(frame_synchronizer, frame_samples, n);
  }

  for(n = 0; n < delay; n++)
  {
    samples[n] = 0;
  }
  msresamp_crcf_execute(resampler, samples, delay, frame_samples, &n);
  gmskframesync_execute(frame_synchronizer, frame_samples, n);

  free(samples);
  free(frame_samples);
  iirfilt_crcf_destroy(low_pass);
  nco_crcf_destroy(oscillator);
  msresamp_crcf_destroy(resampler);
  gmskframesync_destroy(frame_synchronizer);
}

void signal_handler(int signum)
{
  if(verbose)
  {
    fprintf(stderr, "\nStopping (signal %d)\n", signum);
  }
  stop = 1;
  fclose(stdin);
}

void usage()
{
  printf("gmsk-transfer version 1.0.0\n");
  printf("\n");
  printf("Usage: gmsk-transfer [options] [filename]\n");
  printf("\n");
  printf("Options:\n");
  printf("  -b <baud rate>  [default: 9600 b/s]\n");
  printf("    Baud rate of the GMSK transmission.\n");
  printf("  -c <ppm>  [default: 0.0, can be negative]\n");
  printf("    Correction for the radio clock.\n");
  printf("  -d <filename>\n");
  printf("    Dump a copy of the samples sent to or received from\n");
  printf("    the radio (after filtering).\n");
  printf("  -f <frequency>  [default: 434000000 Hz]\n");
  printf("    Frequency of the GMSK transmission.\n");
  printf("  -g <gain>  [default: 0]\n");
  printf("    Gain of the radio transceiver.\n");
  printf("  -h\n");
  printf("    This help.\n");
  printf("  -o <offset>  [default: 0 Hz, can be negative]\n");
  printf("    Set the central frequency of the transceiver 'offset' Hz\n");
  printf("    lower than the signal frequency to send or receive.\n");
  printf("  -r <radio type>  [default: hackrf]\n");
  printf("    Type of radio to use.\n");
  printf("    Supported types:\n");
  printf("      - hackrf: HackRF SDR\n");
  printf("      - io: standard input/output\n");
  printf("  -s <sample rate>  [default: 8000000 S/s]\n");
  printf("    Sample rate to use.\n");
  printf("  -t\n");
  printf("    Use transmit mode.\n");
  printf("  -v\n");
  printf("    Print debug messages:\n");
  printf("      - H: corrupted header\n");
  printf("      - P: corrupted payload\n");
  printf("      - O: overrun\n");
  printf("      - U: underrun\n");
  printf("\n");
  printf("By default the program is in 'receive' mode.\n");
  printf("Use the '-t' option to use the 'transmit' mode.\n");
  printf("\n");
  printf("In 'receive' mode, the samples are received from the radio,\n");
  printf("and the decoded data is written either to 'filename' if it\n");
  printf("is specified, or to standard output.\n");
  printf("In 'transmit' mode, the data to send is read either from\n");
  printf("'filename' if it is specified, or from standard input,\n");
  printf("and the samples are sent to the radio.\n");
  printf("\n");
  printf("Instead of a real radio transceiver, the 'io' radio type uses\n");
  printf("standard input in 'receive' mode, and standard output in\n");
  printf("'transmit' mode. The samples must be in 'float complex' format\n");
  printf("(32 bits for the real part, 32 bits for the imaginary part).\n");
}

int main(int argc, char **argv)
{
  int opt;
  float sample_rate = 8000000;
  float baud_rate = 9600;
  radio_type_t radio_type = HACKRF;
  radio_t radio;
  unsigned int emit = 0;
  unsigned int gain = 0;
  int offset = 0;
  float ppm = 0; // 11;

  radio.frequency = 434000000;

  while((opt = getopt(argc, argv, "b:c:d:f:g:ho:r:s:tv")) != -1)
  {
    switch(opt)
    {
    case 'b':
      baud_rate = strtoul(optarg, NULL, 10);
      break;

    case 'c':
      ppm = strtof(optarg, NULL);
      break;

    case 'd':
      dump = fopen(optarg, "wb");
      if(dump == NULL)
      {
        fprintf(stderr, "Error: Failed to open '%s'\n", optarg);
        return(-1);
      }
      break;

    case 'f':
      radio.frequency = strtoul(optarg, NULL, 10);
      break;

    case 'g':
      gain = strtoul(optarg, NULL, 10);
      break;

    case 'h':
      usage();
      return(0);

    case 'o':
      offset = strtol(optarg, NULL, 10);
      break;

    case 's':
      sample_rate = strtoul(optarg, NULL, 10);
      break;

    case 'r':
      if(strcasecmp(optarg, "io") == 0)
      {
        radio_type = IO;
      }
      else if(strcasecmp(optarg, "hackrf") == 0)
      {
        radio_type = HACKRF;
      }
      else
      {
        fprintf(stderr, "Error: Unknown radio type: '%s'\n", optarg);
        return(-1);
      }
      break;

    case 't':
      emit = 1;
      break;

    case 'v':
      verbose = 1;
      break;

    default:
      fprintf(stderr, "Error: Unknown parameter: '-%c %s'\n", opt, optarg);
      return(-1);
    }
  }
  if(optind < argc)
  {
    if(emit)
    {
      file = fopen(argv[optind], "rb");
    }
    else
    {
      file = fopen(argv[optind], "wb");
    }
    if(file == NULL)
    {
      fprintf(stderr, "Error: Failed to open '%s'\n", argv[optind]);
      return(-1);
    }
  }
  else
  {
    if(emit)
    {
      file = stdin;
    }
    else
    {
      file = stdout;
    }
  }

  if(ppm != 0)
  {
    sample_rate = sample_rate * ((1000000.0 - ppm) / 1000000.0);
    radio.frequency = (unsigned long int) (radio.frequency * ((1000000.0 - ppm) / 1000000.0));
  }
  radio.center_frequency = radio.frequency - offset;

  signal(SIGINT, &signal_handler);
  signal(SIGTERM, &signal_handler);
  signal(SIGABRT, &signal_handler);

  switch(radio_type)
  {
  case IO:
    radio.type = IO;
    if(emit)
    {
      send_frames(&radio, sample_rate, baud_rate);
    }
    else
    {
      receive_frames(&radio, sample_rate, baud_rate);
    }
    break;

  case HACKRF:
    radio.type = HACKRF;
    radio.buffer = cbuffercf_create(sample_rate);
    pthread_mutex_init(&radio.buffer_mutex, NULL);
    HACKRF_CHECK(hackrf_init());
    HACKRF_CHECK(hackrf_open(&radio.device.hackrf));
    HACKRF_CHECK(hackrf_set_amp_enable(radio.device.hackrf, 0));
    HACKRF_CHECK(hackrf_set_sample_rate(radio.device.hackrf, sample_rate));
    HACKRF_CHECK(hackrf_set_baseband_filter_bandwidth(radio.device.hackrf, hackrf_compute_baseband_filter_bw(sample_rate)));
    HACKRF_CHECK(hackrf_set_freq(radio.device.hackrf, radio.center_frequency));
    if(emit)
    {
      HACKRF_CHECK(hackrf_set_txvga_gain(radio.device.hackrf, gain));
      HACKRF_CHECK(hackrf_start_tx(radio.device.hackrf, hackrf_sample_block_requested, (void *) &radio));
      send_frames(&radio, sample_rate, baud_rate);
      HACKRF_CHECK(hackrf_stop_tx(radio.device.hackrf));
    }
    else
    {
      HACKRF_CHECK(hackrf_set_lna_gain(radio.device.hackrf, gain));
      HACKRF_CHECK(hackrf_set_vga_gain(radio.device.hackrf, 20));
      HACKRF_CHECK(hackrf_start_rx(radio.device.hackrf, hackrf_sample_block_received, (void *) &radio));
      receive_frames(&radio, sample_rate, baud_rate);
      HACKRF_CHECK(hackrf_stop_rx(radio.device.hackrf));
    }
    HACKRF_CHECK(hackrf_close(radio.device.hackrf));
    HACKRF_CHECK(hackrf_exit());
    pthread_mutex_destroy(&radio.buffer_mutex);
    cbuffercf_destroy(radio.buffer);
    break;

  default:
    fprintf(stderr, "Error: Unknown radio type\n");
    return(-1);
  }

  fclose(file);
  if(dump)
  {
    fclose(dump);
  }
  if(verbose)
  {
    fprintf(stderr, "\n");
  }

  return(0);
}
