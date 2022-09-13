/*
This file is part of gmsk-transfer, a program to send or receive data
by software defined radio using the GMSK modulation.

Copyright 2022 Guillaume LE VAILLANT

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

#ifndef GMSKFRAMESYNC_H
#define GMSKFRAMESYNC_H

#include <liquid/liquid.h>

// create GMSK frame synchronizer
//  _k          :   samples/symbol
//  _m          :   filter delay (symbols)
//  _BT         :   excess bandwidth factor
//  _dphi_max   :   maximum carrier offset allowable
//  _callback   :   callback function
//  _userdata   :   user data pointer passed to callback function
gmskframesync gmskframesync_create_set2(unsigned int       _k,
                                        unsigned int       _m,
                                        float              _BT,
                                        float              _dphi_max,
                                        framesync_callback _callback,
                                        void *             _userdata);

#endif
