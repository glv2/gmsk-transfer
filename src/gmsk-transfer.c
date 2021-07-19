/*
This file is part of gmsk-transfer, a program to send or receive data
by software defined radio using the GMSK modulation.

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
#include <fcntl.h>
#include <liquid/liquid.h>
#include <math.h>
#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include "gmsk-transfer.h"

#define TAU (2 * M_PI)
#define SAMPLES_PER_SYMBOL 2
#define BT 0.5

#define SOAPYSDR_CHECK(funcall) \
{ \
  int e = funcall; \
  if(e != 0) \
  { \
    fprintf(stderr, "Error: %s\n", SoapySDRDevice_lastError()); \
    exit(EXIT_FAILURE); \
  } \
}

typedef enum
  {
    IO,
    FILENAME,
    SOAPYSDR
  } radio_type_t;

typedef union
{
  FILE *file;
  SoapySDRDevice *soapysdr;
} radio_device_t;

typedef union
{
  SoapySDRStream *soapysdr;
} radio_stream_t;

struct gmsk_transfer_s
{
  radio_type_t radio_type;
  radio_device_t radio_device;
  radio_stream_t radio_stream;
  unsigned char emit;
  FILE *file;
  unsigned long int sample_rate;
  unsigned int bit_rate;
  unsigned long int frequency;
  long int frequency_offset;
  crc_scheme crc;
  fec_scheme inner_fec;
  fec_scheme outer_fec;
  char id[5];
  FILE *dump;
  unsigned char stop;
  int (*data_callback)(void *, unsigned char *, unsigned int);
  void *callback_context;
};

unsigned char stop = 0;
unsigned char verbose = 0;

void gmsk_transfer_set_verbose(unsigned char v)
{
  verbose = v;
}

unsigned char gmsk_transfer_is_verbose()
{
  return(verbose);
}

void dump_samples(gmsk_transfer_t transfer,
                  complex float *samples,
                  unsigned int samples_size)
{
  fwrite(samples, sizeof(complex float), samples_size, transfer->dump);
}

int read_data(void *context,
              unsigned char *payload,
              unsigned int payload_size)
{
  gmsk_transfer_t transfer = (gmsk_transfer_t) context;
  int n;

  if(feof(transfer->file))
  {
    return(-1);
  }

  n = fread(payload, 1, payload_size, transfer->file);
  if(n == 0)
  {
    usleep(1);
  }

  return(n);
}

int write_data(void *context,
               unsigned char *payload,
               unsigned int payload_size)
{
  gmsk_transfer_t transfer = (gmsk_transfer_t) context;

  fwrite(payload, 1, payload_size, transfer->file);
  if(transfer->file == stdout)
  {
    fflush(transfer->file);
  }

  return(payload_size);
}

void send_to_radio(gmsk_transfer_t transfer,
                   complex float *samples,
                   unsigned int samples_size,
                   int last)
{
  unsigned int n;
  unsigned int size;
  int flags = 0;
  size_t mask = 0;
  long long int timestamp = 0;
  int r;
  const void *buffers[1];

  if(transfer->dump)
  {
    dump_samples(transfer, samples, samples_size);
  }

  switch(transfer->radio_type)
  {
  case IO:
    fwrite(samples, sizeof(complex float), samples_size, stdout);
    break;

  case FILENAME:
    fwrite(samples,
           sizeof(complex float),
           samples_size,
           transfer->radio_device.file);
    break;

  case SOAPYSDR:
    n = 0;
    while((n < samples_size) && (!stop) && (!transfer->stop))
    {
      buffers[0] = &samples[n];
      size = samples_size - n;
      r = SoapySDRDevice_writeStream(transfer->radio_device.soapysdr,
                                     transfer->radio_stream.soapysdr,
                                     buffers,
                                     size,
                                     &flags,
                                     0,
                                     10000);
      if(r > 0)
      {
        n += r;
      }
    }
    if(last)
    {
      /* Complete the remaining buffer to ensure that SoapySDR
       * will process it */
      size = SoapySDRDevice_getStreamMTU(transfer->radio_device.soapysdr,
                                         transfer->radio_stream.soapysdr);
      bzero(samples, samples_size * sizeof(complex float));
      buffers[0] = samples;
      while((size > 0) && (!stop) && (!transfer->stop))
      {
        n = (samples_size < size) ? samples_size : size;
        r = SoapySDRDevice_writeStream(transfer->radio_device.soapysdr,
                                       transfer->radio_stream.soapysdr,
                                       buffers,
                                       n,
                                       &flags,
                                       0,
                                       10000);
        if(r > 0)
        {
          size -= r;
        }
      }
      flags = SOAPY_SDR_END_BURST;
      do
      {
        r = SoapySDRDevice_readStreamStatus(transfer->radio_device.soapysdr,
                                            transfer->radio_stream.soapysdr,
                                            &mask,
                                            &flags,
                                            &timestamp,
                                            10000);
      }
      while((r != SOAPY_SDR_UNDERFLOW) && (!stop) && (!transfer->stop));

      /* Give enough time to the hardware to send the last samples */
      usleep(1000000);
    }
    break;
  }
}

unsigned int receive_from_radio(gmsk_transfer_t transfer,
                                complex float *samples,
                                unsigned int samples_size)
{
  unsigned int n = 0;
  int flags;
  long long int timestamp;
  int r;
  void *buffers[1];

  switch(transfer->radio_type)
  {
  case IO:
    n = fread(samples, sizeof(complex float), samples_size, stdin);
    break;

  case FILENAME:
    n = fread(samples,
              sizeof(complex float),
              samples_size,
              transfer->radio_device.file);
    break;

  case SOAPYSDR:
    buffers[0] = samples;
    r = SoapySDRDevice_readStream(transfer->radio_device.soapysdr,
                                  transfer->radio_stream.soapysdr,
                                  buffers,
                                  samples_size,
                                  &flags,
                                  &timestamp,
                                  10000);
    if(r >= 0)
    {
      n = r;
    }
    break;
  }
  return(n);
}

void set_counter(unsigned char *header, unsigned int counter)
{
  header[4] = (counter >> 24) & 255;
  header[5] = (counter >> 16) & 255;
  header[6] = (counter >> 8) & 255;
  header[7] = counter & 255;
}

unsigned int get_counter(unsigned char *header)
{
  return((header[4] << 24) | (header[5] << 16) | (header[6] << 8) | header[7]);
}

void send_dummy_samples(gmsk_transfer_t transfer,
                        msresamp_crcf resampler,
                        nco_crcf oscillator,
                        complex float *frame_samples,
                        unsigned int frame_samples_size,
                        complex float *samples,
                        int last)
{
  unsigned int n;

  for(n = 0; n < frame_samples_size; n++)
  {
    frame_samples[n] = 0;
  }
  msresamp_crcf_execute(resampler, frame_samples, frame_samples_size, samples, &n);
  if(transfer->frequency_offset != 0)
  {
    nco_crcf_mix_block_up(oscillator, samples, samples, n);
  }
  send_to_radio(transfer, samples, n, last);
}

void send_frames(gmsk_transfer_t transfer)
{
  gmskframegen frame_generator = gmskframegen_create();
  float resampling_ratio = (float) transfer->sample_rate / (transfer->bit_rate *
                                                            SAMPLES_PER_SYMBOL);
  msresamp_crcf resampler = msresamp_crcf_create(resampling_ratio, 60);
  unsigned int delay = ceilf(msresamp_crcf_get_delay(resampler));
  unsigned int header_size = 8;
  unsigned char header[header_size];
  unsigned int payload_size = 1000;
  int r;
  unsigned int n;
  unsigned int i;
  /* Process data by blocks of 50 ms */
  unsigned int frame_samples_size = (transfer->bit_rate * SAMPLES_PER_SYMBOL) / 20;
  unsigned int samples_size = ceilf((frame_samples_size + delay) * resampling_ratio);
  int frame_complete;
  float center_frequency = (float) transfer->frequency_offset / transfer->sample_rate;
  nco_crcf oscillator = nco_crcf_create(LIQUID_NCO);
  float maximum_amplitude = 1;
  unsigned int counter = 0;
  unsigned char *payload = malloc(payload_size);
  complex float *frame_samples = malloc(frame_samples_size * sizeof(complex float));
  complex float *samples = malloc(samples_size * sizeof(complex float));

  if((payload == NULL) || (frame_samples == NULL) || (samples == NULL))
  {
    fprintf(stderr, "Error: Memory allocation failed\n");
    exit(EXIT_FAILURE);
  }

  nco_crcf_set_phase(oscillator, 0);
  nco_crcf_set_frequency(oscillator, TAU * center_frequency);

  gmskframegen_set_header_len(frame_generator, header_size);
  memcpy(header, transfer->id, 4);
  set_counter(header, counter);

  while((!stop) && (!transfer->stop))
  {
    r = transfer->data_callback(transfer->callback_context, payload, payload_size);
    if(r < 0)
    {
      break;
    }
    n = r;

    if(n > 0)
    {
      gmskframegen_assemble(frame_generator,
                            header,
                            payload,
                            n,
                            transfer->crc,
                            transfer->inner_fec,
                            transfer->outer_fec);
      frame_complete = 0;
      n = 0;
      while(!frame_complete)
      {
        frame_complete = gmskframegen_write_samples(frame_generator,
                                                    &frame_samples[n]);
        n += SAMPLES_PER_SYMBOL;
        if(frame_complete || (n + SAMPLES_PER_SYMBOL > frame_samples_size))
        {
          /* Reduce the amplitude of samples a little because the resampler
           * may produce samples with an amplitude slightly greater than 1.0
           * otherwise */
          for(i = 0; i < n; i++)
          {
            if(cabsf(frame_samples[i]) > maximum_amplitude)
            {
              maximum_amplitude = cabsf(frame_samples[i]);
            }
          }
          liquid_vectorcf_mulscalar(frame_samples,
                                    n,
                                    0.9 / maximum_amplitude,
                                    frame_samples);
          msresamp_crcf_execute(resampler, frame_samples, n, samples, &n);
          if(transfer->frequency_offset != 0)
          {
            nco_crcf_mix_block_up(oscillator, samples, samples, n);
          }
          send_to_radio(transfer, samples, n, 0);
          n = 0;
          if(frame_complete)
          {
            counter++;
            set_counter(header, counter);
          }
        }
      }
    }
    else
    {
      /* Underrun when reading from stdin. Send some dummy samples to get the
       * remaining output samples for the end of current frame (because of
       * resampler and filter delays) and send them */
      send_dummy_samples(transfer,
                         resampler,
                         oscillator,
                         frame_samples,
                         frame_samples_size,
                         samples,
                         0);
    }
  }

  /* Send some dummy samples to get the remaining output samples (because of
   * resampler and filter delays) */
  send_dummy_samples(transfer,
                     resampler,
                     oscillator,
                     frame_samples,
                     frame_samples_size,
                     samples,
                     1);

  free(samples);
  free(frame_samples);
  free(payload);
  nco_crcf_destroy(oscillator);
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
  gmsk_transfer_t transfer = (gmsk_transfer_t) user_data;
  char id[5];
  unsigned int counter;

  memcpy(id, header, 4);
  id[4] = '\0';
  counter = get_counter(header);

  if(!header_valid || !payload_valid)
  {
    if(verbose)
    {
      if(!header_valid)
      {
        fprintf(stderr, "Frame %u for '%s': corrupted header\n", counter, id);
      }
      if(!payload_valid)
      {
        fprintf(stderr, "Frame %u for '%s': corrupted payload\n", counter, id);
      }
      fflush(stderr);
    }
  }
  if(memcmp(id, transfer->id, 4) != 0)
  {
    if(verbose)
    {
      fprintf(stderr, "Frame %u for '%s': ignored\n", counter, id);
      fflush(stderr);
    }
  }
  else
  {
    transfer->data_callback(transfer->callback_context, payload, payload_size);
  }
  return(0);
}

void receive_frames(gmsk_transfer_t transfer)
{
  gmskframesync frame_synchronizer = gmskframesync_create(frame_received, transfer);
  float resampling_ratio = (transfer->bit_rate *
                            SAMPLES_PER_SYMBOL) / (float) transfer->sample_rate;
  msresamp_crcf resampler = msresamp_crcf_create(resampling_ratio, 60);
  unsigned int delay = ceilf(msresamp_crcf_get_delay(resampler));
  unsigned int n;
  /* Process data by blocks of 50 ms */
  unsigned int frame_samples_size = (transfer->bit_rate * SAMPLES_PER_SYMBOL) / 20;
  unsigned int samples_size = floorf(frame_samples_size / resampling_ratio) + delay;
  nco_crcf oscillator = nco_crcf_create(LIQUID_NCO);
  complex float *frame_samples = malloc((frame_samples_size + delay) *
                                        sizeof(complex float));
  complex float *samples = malloc(samples_size * sizeof(complex float));

  if((frame_samples == NULL) || (samples == NULL))
  {
    fprintf(stderr, "Error: Memory allocation failed\n");
    exit(EXIT_FAILURE);
  }

  nco_crcf_set_phase(oscillator, 0);
  nco_crcf_set_frequency(oscillator, TAU * ((float) transfer->frequency_offset /
                                            transfer->sample_rate));

  while((!stop) && (!transfer->stop))
  {
    n = receive_from_radio(transfer, samples, samples_size);
    if((n == 0) &&
       ((transfer->radio_type == IO) || (transfer->radio_type == FILENAME)))
    {
      break;
    }
    if(transfer->dump)
    {
      dump_samples(transfer, samples, n);
    }
    if(transfer->frequency_offset != 0)
    {
      nco_crcf_mix_block_down(oscillator, samples, samples, n);
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
  nco_crcf_destroy(oscillator);
  msresamp_crcf_destroy(resampler);
  gmskframesync_destroy(frame_synchronizer);
}

gmsk_transfer_t gmsk_transfer_create_callback(char *radio_driver,
                                              unsigned char emit,
                                              int (*data_callback)(void *,
                                                                   unsigned char *,
                                                                   unsigned int),
                                              void *callback_context,
                                              unsigned long int sample_rate,
                                              unsigned int bit_rate,
                                              unsigned long int frequency,
                                              long int frequency_offset,
                                              unsigned int gain,
                                              float ppm,
                                              char *inner_fec,
                                              char *outer_fec,
                                              char *id,
                                              char *dump)
{
  int direction;
  gmsk_transfer_t transfer = malloc(sizeof(struct gmsk_transfer_s));

  if(transfer == NULL)
  {
    fprintf(stderr, "Error: Memory allocation failed\n");
    return(NULL);
  }
  bzero(transfer, sizeof(struct gmsk_transfer_s));

  if(strcasecmp(radio_driver, "io") == 0)
  {
    transfer->radio_type = IO;
  }
  else if(strncasecmp(radio_driver, "file=", 5) == 0)
  {
    transfer->radio_type = FILENAME;
  }
  else
  {
    transfer->radio_type = SOAPYSDR;
  }

  transfer->stop = 0;
  transfer->emit = emit;
  transfer->file = NULL;
  transfer->data_callback = data_callback;
  transfer->callback_context = callback_context;

  if(sample_rate != 0)
  {
    transfer->sample_rate = sample_rate * ((1000000.0 - ppm) / 1000000.0);
  }
  else
  {
    fprintf(stderr, "Error: Invalid sample\n");
    free(transfer);
    return(NULL);
  }

  if(frequency != 0)
  {
    transfer->frequency = frequency * ((1000000.0 - ppm) / 1000000.0);
  }
  else
  {
    fprintf(stderr, "Error: Invalid frequency\n");
    free(transfer);
    return(NULL);
  }

  transfer->frequency_offset = frequency_offset;

  if(bit_rate != 0)
  {
    transfer->bit_rate = bit_rate;
  }
  else
  {
    fprintf(stderr, "Error: Invalid bit rate\n");
    free(transfer);
    return(NULL);
  }

  transfer->crc = LIQUID_CRC_32;

  transfer->inner_fec = liquid_getopt_str2fec(inner_fec);
  if(transfer->inner_fec == LIQUID_FEC_UNKNOWN)
  {
    fprintf(stderr, "Error: Invalid inner FEC\n");
    free(transfer);
    return(NULL);
  }

  transfer->outer_fec = liquid_getopt_str2fec(outer_fec);
  if(transfer->outer_fec == LIQUID_FEC_UNKNOWN)
  {
    fprintf(stderr, "Error: Invalid outer FEC\n");
    free(transfer);
    return(NULL);
  }

  if(strlen(id) <= 4)
  {
    strcpy(transfer->id, id);
  }
  else
  {
    fprintf(stderr, "Error: Id must be at most 4 bytes long\n");
    free(transfer);
    return(NULL);
  }

  if(dump)
  {
    transfer->dump = fopen(dump, "wb");
    if(transfer->dump == NULL)
    {
      fprintf(stderr, "Error: Failed to open '%s'\n", dump);
      free(transfer);
      return(NULL);
    }
  }
  else
  {
    transfer->dump = NULL;
  }

  switch(transfer->radio_type)
  {
  case IO:
    break;

  case FILENAME:
    if(emit)
    {
      transfer->radio_device.file = fopen(radio_driver + 5, "wb");
    }
    else
    {
      transfer->radio_device.file = fopen(radio_driver + 5, "rb");
    }
    if(transfer->radio_device.file == NULL)
    {
      fprintf(stderr, "Error: Failed to open '%s'\n", radio_driver + 5);
      free(transfer);
      return(NULL);
    }
    break;

  case SOAPYSDR:
    transfer->radio_device.soapysdr = SoapySDRDevice_makeStrArgs(radio_driver);
    if(transfer->radio_device.soapysdr == NULL)
    {
      fprintf(stderr, "Error: %s\n", SoapySDRDevice_lastError());
      free(transfer);
      return(NULL);
    }
    direction = emit ? SOAPY_SDR_TX : SOAPY_SDR_RX;
    SOAPYSDR_CHECK(SoapySDRDevice_setSampleRate(transfer->radio_device.soapysdr,
                                                direction,
                                                0,
                                                transfer->sample_rate));
    SOAPYSDR_CHECK(SoapySDRDevice_setFrequency(transfer->radio_device.soapysdr,
                                               direction,
                                               0,
                                               transfer->frequency - transfer->frequency_offset,
                                               NULL));
    SOAPYSDR_CHECK(SoapySDRDevice_setGain(transfer->radio_device.soapysdr,
                                          direction,
                                          0,
                                          gain));
    transfer->radio_stream.soapysdr = SoapySDRDevice_setupStream(transfer->radio_device.soapysdr,
                                                                 direction,
                                                                 SOAPY_SDR_CF32,
                                                                 NULL,
                                                                 0,
                                                                 NULL);
    if(transfer->radio_stream.soapysdr == NULL)
    {
      fprintf(stderr, "Error: %s\n", SoapySDRDevice_lastError());
      SoapySDRDevice_unmake(transfer->radio_device.soapysdr);
      free(transfer);
      return(NULL);
    }
    break;

  default:
    fprintf(stderr, "Error: Unknown radio type\n");
    free(transfer);
    return(NULL);
    break;
  }

  return(transfer);
}

gmsk_transfer_t gmsk_transfer_create(char *radio_driver,
                                     unsigned char emit,
                                     char *file,
                                     unsigned long int sample_rate,
                                     unsigned int bit_rate,
                                     unsigned long int frequency,
                                     long int frequency_offset,
                                     unsigned int gain,
                                     float ppm,
                                     char *inner_fec,
                                     char *outer_fec,
                                     char *id,
                                     char *dump)
{
  int flags;
  gmsk_transfer_t transfer;

  transfer = gmsk_transfer_create_callback(radio_driver,
                                           emit,
                                           emit ? read_data : write_data,
                                           NULL,
                                           sample_rate,
                                           bit_rate,
                                           frequency,
                                           frequency_offset,
                                           gain,
                                           ppm,
                                           inner_fec,
                                           outer_fec,
                                           id,
                                           dump);
  if(transfer == NULL)
  {
    return(NULL);
  }

  transfer->callback_context = transfer;
  if(file)
  {
    if(emit)
    {
      transfer->file = fopen(file, "rb");
    }
    else
    {
      transfer->file = fopen(file, "wb");
    }
    if(transfer->file == NULL)
    {
      fprintf(stderr, "Error: Failed to open '%s'\n", file);
      free(transfer);
      return(NULL);
    }
  }
  else
  {
    if(emit)
    {
      transfer->file = stdin;
      flags = fcntl(STDIN_FILENO, F_GETFL);
      fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
    else
    {
      transfer->file = stdout;
    }
  }

  return(transfer);
}

void gmsk_transfer_free(gmsk_transfer_t transfer)
{
  if(transfer)
  {
    if(transfer->file)
    {
      fclose(transfer->file);
    }
    if(transfer->dump)
    {
      fclose(transfer->dump);
    }
    switch(transfer->radio_type)
    {
    case IO:
      break;

    case FILENAME:
      fclose(transfer->radio_device.file);
      break;

    case SOAPYSDR:
      SoapySDRDevice_deactivateStream(transfer->radio_device.soapysdr,
                                      transfer->radio_stream.soapysdr,
                                      0,
                                      0);
      SoapySDRDevice_closeStream(transfer->radio_device.soapysdr,
                                 transfer->radio_stream.soapysdr);
      SoapySDRDevice_unmake(transfer->radio_device.soapysdr);
      break;

    default:
      break;
    }
    free(transfer);
  }
}

void gmsk_transfer_start(gmsk_transfer_t transfer)
{
  stop = 0;
  transfer->stop = 0;

  switch(transfer->radio_type)
  {
  case IO:
    if(verbose)
    {
      fprintf(stderr, "Info: Using IO pseudo-radio\n");
    }
    break;

  case FILENAME:
    if(verbose)
    {
      fprintf(stderr, "Info: Using FILENAME pseudo-radio\n");
    }
    break;

  case SOAPYSDR:
    SoapySDRDevice_activateStream(transfer->radio_device.soapysdr,
                                  transfer->radio_stream.soapysdr,
                                  0,
                                  0,
                                  0);
    break;

  default:
    return;
  }

  if(transfer->emit)
  {
    send_frames(transfer);
  }
  else
  {
    receive_frames(transfer);
  }
}

void gmsk_transfer_stop(gmsk_transfer_t transfer)
{
  transfer->stop = 1;
}

void gmsk_transfer_stop_all()
{
  stop = 1;
}

void gmsk_transfer_print_available_radios()
{
  size_t size;
  unsigned int i;
  unsigned int j;
  char *driver;
  char *serial;
  SoapySDRKwargs *devices = SoapySDRDevice_enumerate(NULL, &size);

  if(size == 0)
  {
    printf("  No radio detected\n");
  }
  else
  {
    for(i = 0; i < size; i++)
    {
      driver = NULL;
      serial = NULL;
      for(j = 0; j < devices[i].size; j++)
      {
        if(strcasecmp(devices[i].keys[j], "driver") == 0)
        {
          driver = devices[i].vals[j];
        }
        else if(strcasecmp(devices[i].keys[j], "serial") == 0)
        {
          serial = devices[i].vals[j];
          if(strlen(serial) > 8)
          {
            serial = &serial[strlen(serial) - 8];
          }
        }
      }
      printf("  - driver=%s,serial=%s\n", driver, serial);
    }
  }
  SoapySDRKwargsList_clear(devices, size);
}

void gmsk_transfer_print_available_forward_error_codes()
{
  liquid_print_fec_schemes();
}
