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

#ifndef GMSK_TRANSFER_H
#define GMSK_TRANSFER_H

typedef struct gmsk_transfer_s *gmsk_transfer_t;

/* Set the verbosity level
 *  - v: if not 0, print some debug messages to stderr
 */
void gmsk_transfer_set_verbose(unsigned char v);

/* Get the verbosity level */
unsigned char gmsk_transfer_is_verbose();

/* Initialize a new transfer
 *  - radio_driver: radio to use (e.g. "io" or "driver=hackrf")
 *  - emit: 1 for transmit mode; 0 for receive mode
 *  - file: in transmit mode, read data from this file
 *          in receive mode, write data to this file
 *          if NULL, use stdin or stdout instead
 *  - sample_rate: samples per second
 *  - bit_rate: bits per second
 *  - frequency: center frequency of the transfer in Hertz
 *  - frequency_offset: set the frequency of the radio frequency_offset Hz
 *    lower than the frequency of the transfer (can be negative to set the
 *    frequency higher)
 *  - gain: gain of the radio transceiver
 *  - ppm: correction for the radio clock
 *  - inner_fec: inner forward error correction code to use
 *  - outer_fec: outer forward error correction code to use
 *  - id: transfer id; when receiving, frames with a different id will be
 *        ignored
 *  - dump: if not NULL, write raw samples sent or received to this file
 */
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
                                     char *dump);

/* Cleanup after a finished transfer */
void gmsk_transfer_free(gmsk_transfer_t transfer);

/* Start a transfer and return when finished */
void gmsk_transfer_start(gmsk_transfer_t transfer);

/* Interrupt a transfer */
void gmsk_transfer_stop(gmsk_transfer_t transfer);

/* Interrupt all transfers */
void gmsk_transfer_stop_all();

/* Print list of detected software defined radios */
void gmsk_transfer_print_available_radios();

/* Print list of supported forward error codes */
void gmsk_transfer_print_available_forward_error_codes();

#endif
