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

void gmsk_transfer_set_verbose(unsigned char v);
unsigned char gmsk_transfer_is_verbose();
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
void gmsk_transfer_free(gmsk_transfer_t transfer);
void gmsk_transfer_start(gmsk_transfer_t transfer);
void gmsk_transfer_stop(gmsk_transfer_t transfer);
void gmsk_transfer_stop_all();
void gmsk_transfer_print_available_radios();
void gmsk_transfer_print_available_forward_error_codes();

#endif
