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

typedef struct transfer_s *transfer_t;

void set_verbose(unsigned char v);
unsigned char is_verbose();
transfer_t create_transfer(char *radio_type,
                           char *radio_driver,
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
void free_transfer(transfer_t transfer);
void do_transfer(transfer_t transfer);
void interrupt_transfer();
void print_available_radios();
void print_available_forward_error_codes();

#endif
