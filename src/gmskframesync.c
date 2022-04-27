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


This file includes a variation of the code from the liquid-dsp library to
create a gmskframesync object. The original code has the following license:

Copyright (c) 2007 - 2022 Joseph Gaeddert

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <complex.h>
#include <liquid/liquid.h>
#include <stdlib.h>

#define GMSKFRAME_H_USER_DEFAULT 8
#define GMSKFRAMESYNC_PREFILTER 1

// gmskframesync object structure
struct gmskframesync_s {
#if GMSKFRAMESYNC_PREFILTER
    iirfilt_crcf prefilter;         // pre-demodulation filter
#endif
    unsigned int k;                 // filter samples/symbol
    unsigned int m;                 // filter semi-length (symbols)
    float BT;                       // filter bandwidth-time product
    framesync_callback callback;    // user-defined callback function
    void * userdata;                // user-defined data structure
    framesyncstats_s framesyncstats;// frame statistic object
    framedatastats_s framedatastats;// frame statistic object (packet statistics)

    //
    float complex x_prime;          // received sample state
    float fi_hat;                   // instantaneous frequency estimate

    // timing recovery objects, states
    firpfb_rrrf mf;                 // matched filter decimator
    firpfb_rrrf dmf;                // derivative matched filter decimator
    unsigned int npfb;              // number of filters in symsync
    float pfb_q;                    // filtered timing error
    float pfb_soft;                 // soft filterbank index
    int pfb_index;                  // hard filterbank index
    int pfb_timer;                  // filterbank output flag
    float symsync_out;              // symbol synchronizer output

    // synchronizer objects
    detector_cccf frame_detector;   // pre-demod detector
    float tau_hat;                  // fractional timing offset estimate
    float dphi_hat;                 // carrier frequency offset estimate
    float gamma_hat;                // channel gain estimate
    windowcf buffer;                // pre-demod buffered samples, size: k*(pn_len+m)
    nco_crcf nco_coarse;            // coarse carrier frequency recovery

    // preamble
    unsigned int preamble_len;      // number of symbols in preamble
    float * preamble_pn;            // preamble p/n sequence (known)
    float * preamble_rx;            // preamble p/n sequence (received)

    // header
    unsigned int header_user_len;
    unsigned int header_enc_len;
    unsigned int header_mod_len;
    unsigned char * header_mod;
    unsigned char * header_enc;
    unsigned char * header_dec;
    packetizer p_header;
    int header_valid;

    // payload
    char payload_byte;              // received byte
    crc_scheme check;               // payload validity check
    fec_scheme fec0;                // payload FEC (inner)
    fec_scheme fec1;                // payload FEC (outer)
    unsigned int payload_enc_len;   // length of encoded payload
    unsigned int payload_dec_len;   // payload length (num un-encoded bytes)
    unsigned char * payload_enc;    // payload data (encoded bytes)
    unsigned char * payload_dec;    // payload data (encoded bytes)
    packetizer p_payload;           // payload packetizer
    int payload_valid;              // did payload pass crc?

    // status variables
    enum {
        STATE_DETECTFRAME=0,        // detect frame (seek p/n sequence)
        STATE_RXPREAMBLE,           // receive p/n sequence
        STATE_RXHEADER,             // receive header data
        STATE_RXPAYLOAD,            // receive payload data
    } state;
    unsigned int preamble_counter;  // counter: num of p/n syms received
    unsigned int header_counter;    // counter: num of header syms received
    unsigned int payload_counter;   // counter: num of payload syms received
};

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
                                        void *             _userdata)
{
    gmskframesync q = (gmskframesync) malloc(sizeof(struct gmskframesync_s));
    q->callback = _callback;
    q->userdata = _userdata;
    q->k        = _k;        // samples/symbol
    q->m        = _m;        // filter delay (symbols)
    q->BT       = _BT;      // filter bandwidth-time product

#if GMSKFRAMESYNC_PREFILTER
    // create default low-pass Butterworth filter
    q->prefilter = iirfilt_crcf_create_lowpass(3, 0.5f*(1 + q->BT) / (float)(q->k));
#endif

    unsigned int i;

    // frame detector
    q->preamble_len = 63;
    q->preamble_pn = (float*)malloc(q->preamble_len*sizeof(float));
    q->preamble_rx = (float*)malloc(q->preamble_len*sizeof(float));
    float complex preamble_samples[q->preamble_len*q->k];
    msequence ms = msequence_create(6, 0x6d, 1);
    gmskmod mod = gmskmod_create(q->k, q->m, q->BT);

    for (i=0; i<q->preamble_len + q->m; i++) {
        unsigned char bit = msequence_advance(ms);

        // save p/n sequence
        if (i < q->preamble_len)
            q->preamble_pn[i] = bit ? 1.0f : -1.0f;

        // modulate/interpolate
        if (i < q->m) gmskmod_modulate(mod, bit, &preamble_samples[0]);
        else          gmskmod_modulate(mod, bit, &preamble_samples[(i-q->m)*q->k]);
    }

    gmskmod_destroy(mod);
    msequence_destroy(ms);

    // create frame detector
    float threshold = 0.5f;     // detection threshold
    q->frame_detector = detector_cccf_create(preamble_samples, q->preamble_len*q->k, threshold, _dphi_max);
    q->buffer = windowcf_create(q->k*(q->preamble_len+q->m));

    // create symbol timing recovery filters
    q->npfb = 32;   // number of filters in the bank
    q->mf   = firpfb_rrrf_create_rnyquist( LIQUID_FIRFILT_GMSKRX,q->npfb,q->k,q->m,q->BT);
    q->dmf  = firpfb_rrrf_create_drnyquist(LIQUID_FIRFILT_GMSKRX,q->npfb,q->k,q->m,q->BT);

    // create down-coverters for carrier phase tracking
    q->nco_coarse = nco_crcf_create(LIQUID_NCO);

    // create/allocate header objects/arrays
    q->header_mod = NULL;
    q->header_enc = NULL;
    q->header_dec = NULL;
    q->p_header = NULL;
    gmskframesync_set_header_len(q, GMSKFRAME_H_USER_DEFAULT);

    // create/allocate payload objects/arrays
    q->payload_dec_len = 1;
    q->check           = LIQUID_CRC_32;
    q->fec0            = LIQUID_FEC_NONE;
    q->fec1            = LIQUID_FEC_NONE;
    q->p_payload = packetizer_create(q->payload_dec_len,
                                     q->check,
                                     q->fec0,
                                     q->fec1);
    q->payload_enc_len = packetizer_get_enc_msg_len(q->p_payload);
    q->payload_dec = (unsigned char*) malloc(q->payload_dec_len*sizeof(unsigned char));
    q->payload_enc = (unsigned char*) malloc(q->payload_enc_len*sizeof(unsigned char));

    // reset synchronizer
    gmskframesync_reset(q);

    // reset global data counters
    gmskframesync_reset_framedatastats(q);

    // return synchronizer object
    return q;
}
