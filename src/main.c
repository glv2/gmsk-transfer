/*
This file is part of gmsk-transfer, a program to send or receive data
by software defined radio using the GMSK modulation.

Copyright 2021-2022 Guillaume LE VAILLANT

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

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "gmsk-transfer.h"

void signal_handler(int signum)
{
  if(gmsk_transfer_is_verbose())
  {
    fprintf(stderr, "\nStopping (signal %d)\n", signum);
  }
  else
  {
    fprintf(stderr, "\n");
  }
  gmsk_transfer_stop_all();
}

void usage()
{
  printf("gmsk-transfer version 1.6.2\n");
  printf("\n");
  printf("Usage: gmsk-transfer [options] [filename]\n");
  printf("\n");
  printf("Options:\n");
  printf("  -a\n");
  printf("    Use audio samples instead of IQ samples.\n");
  printf("  -b <bit rate>  (default: 9600 b/s)\n");
  printf("    Bit rate of the GMSK transmission.\n");
  printf("  -c <ppm>  (default: 0.0, can be negative)\n");
  printf("    Correction for the radio clock.\n");
  printf("  -d <filename>\n");
  printf("    Dump a copy of the samples sent to or received from\n");
  printf("    the radio.\n");
  printf("  -e <fec[,fec]>  (default: h128,none)\n");
  printf("    Inner and outer forward error correction codes to use.\n");
  printf("  -f <frequency>  (default: 434000000 Hz)\n");
  printf("    Frequency of the GMSK transmission.\n");
  printf("  -g <gain>  (default: 0)\n");
  printf("    Gain of the radio transceiver.\n");
  printf("  -h\n");
  printf("    This help.\n");
  printf("  -i <id>  (default: \"\")\n");
  printf("    Transfer id (at most 4 bytes). When receiving, the frames\n");
  printf("    with a different id will be ignored.\n");
  printf("  -n <bt>  (default: 0.5)\n");
  printf("    Bandwidth-time parameter of the GMSK modulation.\n");
  printf("  -o <offset>  (default: 0 Hz, can be negative)\n");
  printf("    Set the central frequency of the transceiver 'offset' Hz\n");
  printf("    lower than the signal frequency to send or receive.\n");
  printf("  -r <radio>  (default: \"\")\n");
  printf("    Radio to use.\n");
  printf("  -s <sample rate>  (default: 2000000 S/s)\n");
  printf("    Sample rate to use.\n");
  printf("  -T <timeout>  (default: 0 s)\n");
  printf("    Number of seconds after which reception will be stopped if\n");
  printf("    no frame has been received. A timeout of 0 means no timeout.\n");
  printf("  -t\n");
  printf("    Use transmit mode.\n");
  printf("  -u <maximum deviation>  (default: (bit rate / 100) Hz)\n");
  printf("    Maximum allowable deviation of the center frequency of a\n");
  printf("    received signal. If its deviation if greater, frames will\n");
  printf("    probably not be detected.\n");
  printf("  -v\n");
  printf("    Print debug messages.\n");
  printf("  -w <delay>  (default: 0.0 s)\n");
  printf("    Wait a little before switching the radio off.\n");
  printf("    This can be useful if the hardware needs some time to send\n");
  printf("    the last samples it has buffered.\n");
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
  printf("'transmit' mode.\n");
  printf("The 'file=path-to-file' radio type reads/writes the samples\n");
  printf("from/to 'path-to-file'.\n");
  printf("The IQ samples must be in 'complex float' format\n");
  printf("(32 bits for the real part, 32 bits for the imaginary part).\n");
  printf("The audio samples must be in 'signed integer' format (16 bits).\n");
  printf("\n");
  printf("The gain parameter can be specified either as an integer to set a\n");
  printf("global gain, or as a series of keys and values to set specific\n");
  printf("gains (for example 'LNA=32,VGA=20').\n");
  printf("\n");
  printf("Available radios (via SoapySDR):\n");
  gmsk_transfer_print_available_radios();
  printf("\n");
  printf("Available forward error correction codes:\n");
  gmsk_transfer_print_available_forward_error_codes();
}

void get_fec_schemes(char *str, char *inner_fec, char *outer_fec)
{
  unsigned int size = strlen(str);
  char spec[size + 1];
  char *separation;

  strcpy(spec, str);
  if((separation = strchr(spec, ',')) != NULL)
  {
    *separation = '\0';
  }

  if(strlen(spec) < 32)
  {
    strcpy(inner_fec, spec);
  }
  else
  {
    strcpy(inner_fec, "unknown");
  }

  if(separation != NULL)
  {
    if(strlen(separation + 1) < 32)
    {
      strcpy(outer_fec, separation + 1);
    }
    else
    {
      strcpy(outer_fec, "unknown");
    }
  }
  else
  {
    strcpy(outer_fec, "none");
  }
}

int main(int argc, char **argv)
{
  gmsk_transfer_t transfer;
  char *radio_driver = "";
  unsigned int emit = 0;
  unsigned long int sample_rate = 2000000;
  unsigned int bit_rate = 9600;
  unsigned long int frequency = 434000000;
  long int frequency_offset = 0;
  unsigned int maximum_deviation = 0;
  char *gain = "0";
  float ppm = 0;
  float bt = 0.5;
  char inner_fec[32];
  char outer_fec[32];
  char *id = "";
  char *file = NULL;
  char *dump = NULL;
  float final_delay = 0;
  unsigned int final_delay_sec = 0;
  unsigned int final_delay_usec = 0;
  unsigned int timeout = 0;
  unsigned char audio = 0;
  int opt;

  strcpy(inner_fec, "h128");
  strcpy(outer_fec, "none");

  while((opt = getopt(argc, argv, "ab:c:d:e:f:g:hi:n:o:r:s:T:tu:vw:")) != -1)
  {
    switch(opt)
    {
    case 'a':
      audio = 1;
      break;

    case 'b':
      bit_rate = strtoul(optarg, NULL, 10);
      break;

    case 'c':
      ppm = strtof(optarg, NULL);
      break;

    case 'd':
      dump = optarg;
      break;

    case 'e':
      get_fec_schemes(optarg, inner_fec, outer_fec);
      break;

    case 'f':
      frequency = strtoul(optarg, NULL, 10);
      break;

    case 'g':
      gain = optarg;
      break;

    case 'h':
      usage();
      return(EXIT_SUCCESS);

    case 'i':
      id = optarg;
      break;

    case 'n':
      bt = strtof(optarg, NULL);
      break;

    case 'o':
      frequency_offset = strtol(optarg, NULL, 10);
      break;

    case 'r':
      radio_driver = optarg;
      break;

    case 's':
      sample_rate = strtoul(optarg, NULL, 10);
      break;

    case 'T':
      timeout = strtoul(optarg, NULL, 10);
      break;

    case 't':
      emit = 1;
      break;

    case 'u':
      maximum_deviation = strtoul(optarg, NULL, 10);
      break;

    case 'v':
      gmsk_transfer_set_verbose(1);
      break;

    case 'w':
      final_delay = strtof(optarg, NULL);
      break;

    default:
      fprintf(stderr, "Error: Unknown parameter: '-%c %s'\n", opt, optarg);
      return(EXIT_FAILURE);
    }
  }
  if(optind < argc)
  {
    file = argv[optind];
  }
  else
  {
    file = NULL;
  }

  signal(SIGINT, &signal_handler);
  signal(SIGTERM, &signal_handler);
  signal(SIGABRT, &signal_handler);

  transfer = gmsk_transfer_create(radio_driver,
                                  emit,
                                  file,
                                  sample_rate,
                                  bit_rate,
                                  frequency,
                                  frequency_offset,
                                  maximum_deviation,
                                  gain,
                                  ppm,
                                  bt,
                                  inner_fec,
                                  outer_fec,
                                  id,
                                  dump,
                                  timeout,
                                  audio);
  if(transfer == NULL)
  {
    fprintf(stderr, "Error: Failed to initialize transfer\n");
    return(EXIT_FAILURE);
  }
  gmsk_transfer_start(transfer);
  if(final_delay > 0)
  {
    /* Give enough time to the hardware to send the last samples */
    final_delay_sec = floorf(final_delay);
    final_delay_usec = (final_delay - final_delay_sec) * 1000000;
    if(final_delay_sec > 0)
    {
      sleep(final_delay_sec);
    }
    if(final_delay_usec > 0)
    {
      usleep(final_delay_usec);
    }
  }
  gmsk_transfer_free(transfer);

  if(gmsk_transfer_is_verbose())
  {
    fprintf(stderr, "\n");
  }

  return(EXIT_SUCCESS);
}
