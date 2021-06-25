#!/bin/sh

# This file is part of gmsk-transfer, a program to send or receive data
# by radio using the GMSK modulation.
#
# Copyright 2021 Guillaume LE VAILLANT
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

set -e

MESSAGE=$(mktemp -t message.XXXXXX)
DECODED=$(mktemp -t decoded.XXXXXX)
SAMPLES=$(mktemp -t samples.XXXXXX)

echo "This is a test transmission using gmsk-transfer." > ${MESSAGE}

function check()
{
    NAME=$1
    OPTIONS=$2

    echo "Test: ${NAME}"
    ./gmsk-transfer -t -r io ${OPTIONS} ${MESSAGE} > ${SAMPLES}
    ./gmsk-transfer -r io ${OPTIONS} ${DECODED} < ${SAMPLES}
    diff -q ${MESSAGE} ${DECODED}
}

check "Default parameters" ""
check "Bit rate 1200" "-b 1200"
check "Bit rate 38400" "-b 38400"
check "Bit rate 400000" "-b 400000"
check "Frequency offset 200000" "-o 200000"
check "Frequency offset -123456" "-o -123456"
check "Sample rate 4000000" "-s 4000000"
check "Sample rate 10000000" "-s 10000000"
check "FEC Hamming(7/4)" "-e h74"
check "FEC Golay(24/12) and repeat(3)" "-e g2412,rep3"
check "Id a1B2" "-i a1B2"

rm -f ${MESSAGE} ${DECODED} ${SAMPLES}
echo "All tests passed."
