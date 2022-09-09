# Aligns the rcvTow in RAWX messages in u-blox UBX binary files to the nearest decimalPlaces seconds

# Written by: Paul Clark
# Last update: August 17th 2022

# SparkFun code, firmware, and software is released under the MIT License (http://opensource.org/licenses/MIT)
#
# The MIT License (MIT)
#
# Copyright (c) 2022 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import sys
import os
import struct

# Add byte to checksums sum1 and sum2
def csum(byte, sum1, sum2):
    sum1 = sum1 + byte
    sum2 = sum2 + sum1
    sum1 = sum1 & 0xFF
    sum2 = sum2 & 0xFF
    return sum1,sum2

print('UBX RAWX Aligner')
print()

filename = ''

if filename == '':
    # Check if the bin filename was passed in argv
    if len(sys.argv) > 1: filename = sys.argv[1]

# Find first .ubx file in the current directory
firstfile = ''
for root, dirs, files in os.walk("."):
    if len(files) > 0:
        if root == ".": # Comment this line to check sub-directories too
            for afile in files:
                if afile[-4:] == '.ubx':
                    if firstfile == '': firstfile = os.path.join(root, afile)

# Ask user for .bin filename offering firstfile as the default
if filename == '': filename = input('Enter the UBX filename (default: ' + firstfile + '): ') # Get the filename
if filename == '': filename = firstfile

# Ask user if the data contains NMEA messages
response = input('Could this file contain any NMEA messages? (Y/n): ') # Get the response
if (response == '') or (response == 'Y') or (response == 'y'):
    containsNMEA = True
else:
    containsNMEA = False

repairFile = True
if (filename[-4] == '.'):
    repairFilename = filename[:-4] + '.aligned' + filename[-4:]
else:
    repairFilename = filename + '.aligned'

decimalPlaces = 0 # Default to whole seconds
if len(sys.argv) > 2: decimalPlaces = sys.argv[2]

print()
print('Processing',filename)
print()
filesize = os.path.getsize(filename) # Record the file size

# Try to open file for reading
try:
    fi = open(filename,"rb")
except:
    raise Exception('Invalid file!')

# Try to open repair file for write and read
if (repairFile):
    try:
        fo = open(repairFilename,"w+b")
    except:
        raise Exception('Could not open aligned file!')

processed = -1 # The number of bytes processed
messages = {} # The collected message types
keepGoing = True

# Sync 'state machine'
looking_for_B5_dollar   = 0 # Looking for either a UBX 0xB5 or an NMEA '$'
looking_for_62          = 1 # Looking for a UBX 0x62 header byte
looking_for_class       = 2 # Looking for UBX class byte
looking_for_ID          = 3 # Looking for UBX ID byte
looking_for_length_LSB  = 4 # Looking for UBX length bytes
looking_for_length_MSB  = 5
processing_payload      = 6 # Processing the payload. Keep going until length bytes have been processed
looking_for_checksum_A  = 7 # Looking for UBX checksum bytes
looking_for_checksum_B  = 8
sync_lost               = 9 # Go into this state if sync is lost (bad checksum etc.)
looking_for_asterix     = 10 # Looking for NMEA '*'
looking_for_csum1       = 11 # Looking for NMEA checksum bytes
looking_for_csum2       = 12
looking_for_term1       = 13 # Looking for NMEA terminating bytes (CR and LF)
looking_for_term2       = 14

ubx_nmea_state = sync_lost # Initialize the state machine

# Storage for UBX messages
ubx_length = 0
ubx_length_LSB = 0
ubx_length_MSB = 0
ubx_class = 0
ubx_ID = 0
ubx_checksum_A = 0
ubx_checksum_B = 0
ubx_expected_checksum_A = 0
ubx_expected_checksum_B = 0
longest_UBX = 0 # The length of the longest UBX message
longest_UBX_candidate = 0 # Candidate for the length of the longest valid UBX message

# Storage for NMEA messages
nmea_length = 0
nmea_char_1 = 0 # e.g. G
nmea_char_2 = 0 # e.g. P
nmea_char_3 = 0 # e.g. G
nmea_char_4 = 0 # e.g. G
nmea_char_5 = 0 # e.g. A
nmea_csum = 0
nmea_csum1 = 0
nmea_csum2 = 0
nmea_expected_csum1 = 0
nmea_expected_csum2 = 0
longest_NMEA = 0 # The length of the longest valid NMEA message

max_nmea_len = 128 # Maximum length for an NMEA message: use this to detect if we have lost sync while receiving an NMEA message
sync_lost_at = -1 # Record where we lost sync
rewind_to = -1 # Keep a note of where we should rewind to if sync is lost
rewind_attempts = 0 # Keep a note of how many rewinds have been attempted
max_rewinds = 100 # Abort after this many rewinds
rewind_in_progress = False # Flag to indicate if a rewind is in progress
resyncs = 0 # Record the number of successful resyncs
resync_in_progress = False # Flag to indicate if a resync is in progress
message_start_byte = 0 # Record where the latest message started (for resync reporting)

rewind_repair_file_to = 0 # Keep a note of where to rewind the repair file to if sync is lost
repaired_file_bytes = 0 # Keep a note of how many bytes have been written to the repair file

repair_file_rawx_payload_start = 0 # Keep a note of where the RAWX payload starts (i.e. where the rcvTow R8 starts)
largest_rawx_alignment = 0.0 # Keep note of the largest alignment change

try:
    while keepGoing:

        # Read one byte from the file
        fileBytes = fi.read(1)
        if (len(fileBytes) == 0):
            print('ERROR: Read zero bytes. End of file?! Or zero file size?!')
            raise Exception('End of file?! Or zero file size?!')
        c = fileBytes[0]

        processed = processed + 1 # Keep a record of how many bytes have been read and processed

        # Write the byte to the repair file if desired
        if (repairFile):
            fo.write(fileBytes)
            repaired_file_bytes = repaired_file_bytes + 1

        # Process data bytes according to ubx_nmea_state
        # For UBX messages:
        # Sync Char 1: 0xB5
        # Sync Char 2: 0x62
        # Class byte
        # ID byte
        # Length: two bytes, little endian
        # Payload: length bytes
        # Checksum: two bytes
        # For NMEA messages:
        # Starts with a '$'
        # The next five characters indicate the message type (stored in nmea_char_1 to nmea_char_5)
        # Message fields are comma-separated
        # Followed by an '*'
        # Then a two character checksum (the logical exclusive-OR of all characters between the $ and the * as ASCII hex)
        # Ends with CR LF
        # Only allow a new file to be opened when a complete packet has been processed and ubx_nmea_state has returned to "looking_for_B5_dollar"
        # Or when a data error is detected (sync_lost)

        # RXM_RAWX is class 0x02 ID 0x15
        # RXM_SFRBF is class 0x02 ID 0x13
        # TIM_TM2 is class 0x0d ID 0x03
        # NAV_POSLLH is class 0x01 ID 0x02
        # NAV_PVT is class 0x01 ID 0x07
        # NAV-STATUS is class 0x01 ID 0x03

        if (ubx_nmea_state == looking_for_B5_dollar) or (ubx_nmea_state == sync_lost):
            if (c == 0xB5): # Have we found Sync Char 1 (0xB5) if we were expecting one?
                if (ubx_nmea_state == sync_lost):
                    print("UBX Sync Char 1 (0xB5) found at byte "+str(processed)+". Checking for Sync Char 2")
                ubx_nmea_state = looking_for_62 # Now look for Sync Char 2 (0x62)
                message_start_byte = processed # Record the message start byte for resync reporting
            elif (c == 0x24) and (containsNMEA == True): # Have we found an NMEA '$' if we were expecting one?
                if (ubx_nmea_state == sync_lost):
                    print("NMEA $ found at byte "+str(processed)+". Attempting to process the message")
                ubx_nmea_state = looking_for_asterix # Now keep going until we receive an asterix
                nmea_length = 0 # Reset nmea_length then use it to check for excessive message length
                nmea_csum = 0 # Reset the nmea_csum. Update it as each character arrives
                nmea_char_1 = 0x30 # Reset the first five NMEA chars to something invalid
                nmea_char_2 = 0x30
                nmea_char_3 = 0x30
                nmea_char_4 = 0x30
                nmea_char_5 = 0x30
                message_start_byte = processed # Record the message start byte for resync reporting
            else:
                #print("Was expecting Sync Char 0xB5 or an NMEA $ but did not receive one!")
                if (c == 0x24):
                    print("Warning: * found at byte "+str(processed)+"! Are you sure this file does not contain NMEA messages?")
                sync_lost_at = processed
                ubx_nmea_state = sync_lost
        elif (ubx_nmea_state == looking_for_62):
            if (c == 0x62): # Have we found Sync Char 2 (0x62) when we were expecting one?
                ubx_expected_checksum_A = 0 # Reset the expected checksum
                ubx_expected_checksum_B = 0
                ubx_nmea_state = looking_for_class # Now look for Class byte
            else:
                print("Panic!! Was expecting Sync Char 2 (0x62) but did not receive one!")
                print("Sync lost at byte "+str(processed)+". Attemting to re-sync")
                sync_lost_at = processed
                resync_in_progress = True
                ubx_nmea_state = sync_lost
        elif (ubx_nmea_state == looking_for_class):
            ubx_class = c
            ubx_expected_checksum_A = ubx_expected_checksum_A + c # Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A
            ubx_nmea_state = looking_for_ID # Now look for ID byte
        elif (ubx_nmea_state == looking_for_ID):
            ubx_ID = c
            ubx_expected_checksum_A = ubx_expected_checksum_A + c # Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A
            message_type = '0x%02X 0x%02X'%(ubx_class,ubx_ID) # Record the message type
            ubx_nmea_state = looking_for_length_LSB # Now look for length LSB
        elif (ubx_nmea_state == looking_for_length_LSB):
            ubx_length = c # Store the length LSB
            ubx_length_LSB = c
            ubx_expected_checksum_A = ubx_expected_checksum_A + c # Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A
            ubx_nmea_state = looking_for_length_MSB # Now look for length MSB
        elif (ubx_nmea_state == looking_for_length_MSB):
            ubx_length = ubx_length + (c * 256) # Add the length MSB
            ubx_length_MSB = c
            ubx_expected_checksum_A = ubx_expected_checksum_A + c # Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A
            longest_UBX_candidate = ubx_length + 8 # Update the longest UBX message length candidate. Include the header, class, ID, length and checksum bytes
            rewind_to = processed # If we lose sync due to dropped bytes then rewind to here
            ubx_nmea_state = processing_payload # Now look for payload bytes (length: ubx_length)

            if (message_type == '0x02 0x15'): # Is this RAWX? If so, record the start of the payload
                repair_file_rawx_payload_start = repaired_file_bytes

        elif (ubx_nmea_state == processing_payload):
            ubx_length = ubx_length - 1 # Decrement length by one
            ubx_expected_checksum_A = ubx_expected_checksum_A + c # Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A
            if (ubx_length == 0):
                ubx_expected_checksum_A = ubx_expected_checksum_A & 0xff # Limit checksums to 8-bits
                ubx_expected_checksum_B = ubx_expected_checksum_B & 0xff
                ubx_nmea_state = looking_for_checksum_A # If we have received length payload bytes, look for checksum bytes
        elif (ubx_nmea_state == looking_for_checksum_A):
            ubx_checksum_A = c
            ubx_nmea_state = looking_for_checksum_B
        elif (ubx_nmea_state == looking_for_checksum_B):
            ubx_checksum_B = c
            ubx_nmea_state = looking_for_B5_dollar # All bytes received so go back to looking for a new Sync Char 1 unless there is a checksum error
            if ((ubx_expected_checksum_A != ubx_checksum_A) or (ubx_expected_checksum_B != ubx_checksum_B)):
                print("Panic!! UBX checksum error!")
                print("Sync lost at byte "+str(processed)+". Attemting to re-sync.")
                sync_lost_at = processed
                resync_in_progress = True
                ubx_nmea_state = sync_lost
            else:
                # Valid UBX message was received. Check if we have seen this message type before
                if message_type in messages:
                    messages[message_type] += 1 # if we have, increment its count
                else:
                    messages[message_type] = 1 # if we have not, set its count to 1
                if (longest_UBX_candidate > longest_UBX): # Update the longest UBX message length
                    longest_UBX = longest_UBX_candidate
                rewind_in_progress = False # Clear rewind_in_progress
                rewind_to = -1
                if (resync_in_progress == True): # Check if we are resyncing
                    resync_in_progress = False # Clear the flag now that a valid message has been received
                    resyncs += 1 # Increment the number of successful resyncs
                    print("Sync successfully re-established at byte "+str(processed)+". The UBX message started at byte "+str(message_start_byte))
                    print()
                    if (repairFile):
                        fo.seek(rewind_repair_file_to) # Rewind the repaired file
                        repaired_file_bytes = rewind_repair_file_to
                        fi.seek(message_start_byte) # Copy the valid message into the repair file
                        repaired_bytes_to_write = processed - message_start_byte
                        fileBytes = fi.read(repaired_bytes_to_write)
                        fo.write(fileBytes)
                        repaired_file_bytes = repaired_file_bytes + repaired_bytes_to_write

                        # Note: alignment is skipped if resyncing is in progress
                        # You may need to run the code twice to align any skipped alignments

                else:
                    if (repairFile):
                        rewind_repair_file_to = repaired_file_bytes # Rewind repair file to here if sync is lost

                        if (message_type == '0x02 0x15'): # Is this RAWX? If so, do the alignment

                            ubx_expected_checksum_A = 0 # Reuse the expected checksum
                            ubx_expected_checksum_B = 0
                            ubx_expected_checksum_A = ubx_expected_checksum_A + ubx_class
                            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A
                            ubx_expected_checksum_A = ubx_expected_checksum_A + ubx_ID
                            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A
                            ubx_expected_checksum_A = ubx_expected_checksum_A + ubx_length_LSB
                            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A
                            ubx_expected_checksum_A = ubx_expected_checksum_A + ubx_length_MSB
                            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A

                            fo.seek(repair_file_rawx_payload_start) # Rewind the repair file
                            
                            fileBytes = fo.read(8) # Read the rcvTow R8
                            rcvTow = struct.unpack('<d', fileBytes)[0] # Unpack the R8 (Little-endian)
                            rcvTowRounded = round(rcvTow, decimalPlaces) # Round to decimalPlaces
                            if (abs(rcvTow - rcvTowRounded) > largest_rawx_alignment):
                                largest_rawx_alignment = abs(rcvTow - rcvTowRounded) # Record the largest alignment change
                            fileBytes = struct.pack('<d', rcvTowRounded)
                            fo.seek(repair_file_rawx_payload_start) # Rewind the repair file
                            fo.write(fileBytes) # Write the rounded TOW to the repair file                        
                            for i in range(8): # Update the checksum
                                ubx_expected_checksum_A = ubx_expected_checksum_A + fileBytes[i]
                                ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A

                            bytesToRead = (ubx_length_MSB * 256) + ubx_length_LSB - 8
                            fileBytes = fo.read(bytesToRead)
                            for i in range(bytesToRead): # Update the checksum
                                ubx_expected_checksum_A = ubx_expected_checksum_A + fileBytes[i]
                                ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A
                            ubx_expected_checksum_A = ubx_expected_checksum_A & 0xff # Limit checksums to 8-bits
                            ubx_expected_checksum_B = ubx_expected_checksum_B & 0xff
                            fileBytes = struct.pack('BB', ubx_expected_checksum_A, ubx_expected_checksum_B)
                            fo.write(fileBytes) # Write the updated checksum

        # NMEA messages
        elif (ubx_nmea_state == looking_for_asterix):
            nmea_length = nmea_length + 1 # Increase the message length count
            if (nmea_length > max_nmea_len): # If the length is greater than max_nmea_len, something bad must have happened (sync_lost)
                print("Panic!! Excessive NMEA message length!")
                print("Sync lost at byte "+str(processed)+". Attemting to re-sync")
                sync_lost_at = processed
                resync_in_progress = True
                ubx_nmea_state = sync_lost
                continue
            # If this is one of the first five characters, store it
            if (nmea_length <= 5):
                if (nmea_length == 1):
                    nmea_char_1 = c
                    rewind_to = processed # If we lose sync due to dropped bytes then rewind to here
                elif (nmea_length == 2):
                    nmea_char_2 = c
                elif (nmea_length == 3):
                    nmea_char_3 = c
                elif (nmea_length == 4):
                    nmea_char_4 = c
                else: # ubx_length == 5
                    nmea_char_5 = c
                    message_type = chr(nmea_char_1) + chr(nmea_char_2) + chr(nmea_char_3) + chr(nmea_char_4) + chr(nmea_char_5) # Record the message type
                    if (message_type == "PUBX,"): # Remove the comma from PUBX
                        message_type = "PUBX"
            # Now check if this is an '*'
            if (c == 0x2A):
                # Asterix received
                # Don't exOR it into the checksum
                # Instead calculate what the expected checksum should be (nmea_csum in ASCII hex)
                nmea_expected_csum1 = ((nmea_csum & 0xf0) >> 4) + 0x30 # Convert MS nibble to ASCII hex
                if (nmea_expected_csum1 >= 0x3A): # : follows 9 so add 7 to convert to A-F
                    nmea_expected_csum1 += 7
                nmea_expected_csum2 = (nmea_csum & 0x0f) + 0x30 # Convert LS nibble to ASCII hex
                if (nmea_expected_csum2 >= 0x3A): # : follows 9 so add 7 to convert to A-F
                    nmea_expected_csum2 += 7
                # Next, look for the first csum character
                ubx_nmea_state = looking_for_csum1
                continue # Don't include the * in the checksum
            # Now update the checksum
            # The checksum is the exclusive-OR of all characters between the $ and the *
            nmea_csum = nmea_csum ^ c
        elif (ubx_nmea_state == looking_for_csum1):
            # Store the first NMEA checksum character
            nmea_csum1 = c
            ubx_nmea_state = looking_for_csum2
        elif (ubx_nmea_state == looking_for_csum2):
            # Store the second NMEA checksum character
            nmea_csum2 = c
            # Now check if the checksum is correct
            if ((nmea_csum1 != nmea_expected_csum1) or (nmea_csum2 != nmea_expected_csum2)):
                # The checksum does not match so sync_lost
                print("Panic!! NMEA checksum error!")
                print("Sync lost at byte "+str(processed)+". Attemting to re-sync")
                sync_lost_at = processed
                resync_in_progress = True
                ubx_nmea_state = sync_lost
            else:
                # Checksum was valid so wait for the terminators
                ubx_nmea_state = looking_for_term1
        elif (ubx_nmea_state == looking_for_term1):
            # Check if this is CR
            if (c != 0x0D):
                print("Panic!! NMEA CR not found!")
                print("Sync lost at byte "+str(processed)+". Attemting to re-sync")
                sync_lost_at = processed
                resync_in_progress = True
                ubx_nmea_state = sync_lost
            else:
                ubx_nmea_state = looking_for_term2
        elif (ubx_nmea_state == looking_for_term2):
            # Check if this is LF
            if (c != 0x0A):
                print("Panic!! NMEA LF not found!")
                print("Sync lost at byte "+str(processed)+". Attemting to re-sync")
                sync_lost_at = processed
                resync_in_progress = True
                ubx_nmea_state = sync_lost
            else:
                # Valid NMEA message was received. Check if we have seen this message type before
                if message_type in messages:
                    messages[message_type] += 1 # if we have, increment its count
                else:
                    messages[message_type] = 1 # if we have not, set its count to 1
                if (nmea_length > longest_NMEA): # Update the longest NMEA message length
                    longest_NMEA = nmea_length
                # LF was received so go back to looking for B5 or a $
                ubx_nmea_state = looking_for_B5_dollar
                rewind_in_progress = False # Clear rewind_in_progress
                rewind_to = -1
                if (resync_in_progress == True): # Check if we are resyncing
                    resync_in_progress = False # Clear the flag now that a valid message has been received
                    resyncs += 1 # Increment the number of successful resyncs
                    print("Sync successfully re-established at byte "+str(processed)+". The NMEA message started at byte "+str(message_start_byte))
                    print()
                    if (repairFile):
                        fo.seek(rewind_repair_file_to) # Rewind the repaired file
                        repaired_file_bytes = rewind_repair_file_to
                        fi.seek(message_start_byte) # Copy the valid message into the repair file
                        repaired_bytes_to_write = processed - message_start_byte
                        fileBytes = fi.read(repaired_bytes_to_write)
                        fo.write(fileBytes)
                        repaired_file_bytes = repaired_file_bytes + repaired_bytes_to_write
                else:
                    if (repairFile):
                        rewind_repair_file_to = repaired_file_bytes # Rewind repair file to here if sync is lost

        # Check if the end of the file has been reached
        if (processed >= filesize - 1): keepGoing = False

        # Check if we should attempt to rewind
        # Don't rewind if we have not yet seen a valid message
        # Don't rewind if a rewind is already in progress
        if (ubx_nmea_state == sync_lost) and (len(messages) > 0) and (rewind_in_progress == False) and (rewind_to >= 0):
            rewind_attempts += 1 # Increment the number of rewind attempts
            if (rewind_attempts > max_rewinds): # Only rewind up to max_rewind times
                print("Panic! Maximum rewind attempts reached! Aborting...")
                keepGoing = False
            else:
                print("Sync has been lost. Currently processing byte "+str(processed)+". Rewinding to byte "+str(rewind_to))
                fi.seek(rewind_to) # Rewind the file
                processed = rewind_to - 1 # Rewind processed too! (-1 is needed as processed is incremented at the start of the loop)
                rewind_in_progress = True # Flag that a rewind is in progress
            

finally:
    fi.close() # Close the file

    if (repairFile):
        fo.close()
        
    # Print the file statistics
    print()
    processed += 1
    print('Processed',processed,'bytes')
    print('File size was',filesize)
    if (processed != filesize):
        print('FILE SIZE MISMATCH!!')
    print('Longest valid UBX message was %i bytes'%longest_UBX)
    if (containsNMEA == True):
        print('Longest valid NMEA message was %i characters'%longest_NMEA)
    if len(messages) > 0:
        print('Message types and totals were:')
        for key in messages.keys():
            print('Message type:',key,'  Total:',messages[key])
    if (resyncs > 0):
        print('Number of successful resyncs:',resyncs)
    print()
    if (repairFile):
        print('Aligned data written to:', repairFilename)
        print('Largest alignment change:', largest_rawx_alignment)
        if (resyncs > 0):
            print('Note: alignment is skipped during resyncing')
            print('You may need to run the code twice to align any skipped alignments')
    print()
    print('Bye!')
