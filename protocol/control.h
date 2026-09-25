/*
 * Control protocol shared by the ESP32-S3 firmware, the FPGA and the host.
 *
 * Both devices speak the same framing over their serial ports:
 *   ESP32-S3: native USB Serial/JTAG (baud rate irrelevant)
 *   FPGA:     FT2232 channel B UART, 1 Mbaud 8N1
 *
 * Request  (10 bytes): magic 0xB4, op, arg:u16, seq:u16, crc32(bytes 0..5)
 * Response (16 bytes): magic 0xB5, node, op, status, seq:u16, reserved:u16,
 *                      value:u32, crc32(bytes 0..11)
 *
 * All multi-byte fields are little-endian. CRC32 is IEEE 802.3 (reflected,
 * polynomial 0xEDB88320, initial value and final XOR 0xFFFFFFFF).
 *
 * This file is also translated into Verilog `defines by the FPGA build, so
 * every constant must stay a plain "#define NAME <decimal or 0x hex>".
 */
#ifndef IQSTREAM_CONTROL_H
#define IQSTREAM_CONTROL_H

#define CTL_REQUEST_MAGIC 0xB4
#define CTL_RESPONSE_MAGIC 0xB5
#define CTL_REQUEST_BYTES 10
#define CTL_RESPONSE_BYTES 16

#define CTL_NODE_ESP 1
#define CTL_NODE_FPGA 2

/* Response status codes. */
#define CTL_OK 0
#define CTL_UNKNOWN_OP 1
#define CTL_BAD_ARGUMENT 2
#define CTL_BUSY 3
#define CTL_NOT_READY 4
#define CTL_RUN_FAILED 5
#define CTL_FAILED 6      /* attempted and failed; see the operation */

/* Identity values returned by CTL_INFO with argument 0. */
#define CTL_ESP_FIRMWARE_ID 0x49515302
#define CTL_FPGA_FIRMWARE_ID 0x49514603

/* Operations understood by both nodes. */
#define CTL_INFO 1   /* arg 0: firmware id; ESP arg 1/2: MAC bytes 0-3 / 4-5 */
#define CTL_SAFE 2   /* abort any activity and release every link output */
#define CTL_STATUS 3 /* arg: statistic index (ESP_STAT_* / FPGA_STAT_*) */

/* ESP32-S3 operations. */
#define ESP_OUTPUTS 16 /* arg 1: drive all 16 link lines low; arg 0: release */
#define ESP_RUN 17     /* arg: seconds (1..65535), or 0 to run until stopped */
#define ESP_STOP 18    /* sent while RUN is active: finish the current unit pair */
#define ESP_ARG_HIGH 19 /* arg: bits 16..31 of the next operation's argument */

/*
 * Receiver settings. Each applies at once (between runs; RUN keeps the ESP
 * busy) and is answered with the value now in effect, in the same encoding.
 * Any change reconfigures the whole receive path the way it is set up at
 * boot, so the other settings' ESP_STAT_* values can change with it (an
 * automatic gain stage, for example). A setting that is refused (bad
 * argument) or fails (CTL_FAILED) leaves the previous configuration.
 */
#define ESP_SET_LO 20       /* LO frequency, Hz (with ESP_ARG_HIGH); reply: exact LO */
#define ESP_SET_RATE 21     /* ESP_RATE_* */
#define ESP_SET_WIDTH 22    /* analog channel width, MHz: 20 or 40 */
#define ESP_SET_FILTER 23   /* baseband RC filter codes: first | second << 8, each 0..63 */
#define ESP_SET_GAIN 24     /* receive gain selector 0..127 (an AGC table index, not dB) */
#define ESP_SET_RF_GAIN 25  /* RF gain stage word 0..511, or ESP_AUTO: from the selector */
#define ESP_SET_BB_GAIN 26  /* baseband gain stage, low 7 bits 0..127, or ESP_AUTO */
#define ESP_SET_DC 27       /* DC offset: register << 12 | code 0..511, or | ESP_DC_AUTO */
#define ESP_SET_IQ 28       /* I/Q correction: amplitude | phase << 8, or ESP_AUTO */

#define ESP_AUTO 0x8000     /* setting follows the hardware (calibration, gain table) */
#define ESP_DC_AUTO 0xFFF
#define ESP_RATE_80M 0      /* 80 Msps */
#define ESP_RATE_16M 1      /* 16 Msps */
#define ESP_LO_MIN_HZ 2210000000 /* the RF PLL locks from about 2207 to 2795 MHz; */
#define ESP_LO_MAX_HZ 2790000000 /* reception was verified from 2356 to 2476 MHz */

/*
 * DC offset registers (ESP_SET_DC): 0 and 1 shift I, 2 and 3 shift Q, by
 * about 1.2 and 0.8 ADC counts per code respectively. I/Q correction
 * (ESP_SET_IQ): amplitude is signed 5-bit (about 1/64 per code) and phase
 * signed 6-bit (about 1/128 per code), each as two's complement in its byte.
 * The RF gain word's bits interact; the gain selector's table is the tested
 * way to set both stages.
 */

/*
 * ESP_RUN replies only after acquisition has ended. To end a run early the
 * host sends ESP_STOP: the ESP notices pending input, finishes at the next
 * complete two-unit boundary, answers RUN, and then answers the STOP request.
 * ESP_RUN requires ESP_OUTPUTS 1 first, so the FPGA never sees floating lines.
 */

/* ESP statistics, valid after a run. */
#define ESP_STAT_STATUS 0         /* 0 = success, otherwise ESP_FAIL_* */
#define ESP_STAT_DETAIL 1         /* failure-specific detail word */
#define ESP_STAT_FAIL_LANE 2      /* lane (CPU core) that reported the failure */
#define ESP_STAT_UNITS0 3         /* units transmitted on lane 0 */
#define ESP_STAT_UNITS1 4
#define ESP_STAT_PAIRS0_LO 5      /* IQ pairs transmitted on lane 0 */
#define ESP_STAT_PAIRS0_HI 6
#define ESP_STAT_PAIRS1_LO 7
#define ESP_STAT_PAIRS1_HI 8
#define ESP_STAT_TICKS_LO 9       /* acquisition time, 16 MHz system timer ticks */
#define ESP_STAT_TICKS_HI 10
#define ESP_STAT_SERVICE_MAX0 11  /* worst unit service time on lane 0, CPU cycles */
#define ESP_STAT_SERVICE_MAX1 12
#define ESP_STAT_RADIO 13         /* radio initialisation result, ESP_RADIO_* */
#define ESP_STAT_STOPPED_BY_HOST 14
/* Receiver settings in effect, in their ESP_SET_* encodings. */
#define ESP_STAT_LO_HZ 15
#define ESP_STAT_RATE 16
#define ESP_STAT_WIDTH 17
#define ESP_STAT_FILTER 18
#define ESP_STAT_GAIN 19
#define ESP_STAT_RF_GAIN 20       /* RF stage word in effect */
#define ESP_STAT_BB_GAIN 21       /* baseband stage in effect, low 7 bits */
#define ESP_STAT_DC0 22           /* DC offset codes in effect, registers 0..3 */
#define ESP_STAT_DC1 23
#define ESP_STAT_DC2 24
#define ESP_STAT_DC3 25
#define ESP_STAT_IQ 26            /* I/Q correction fields in the register */
#define ESP_STAT_AUTOMATIC 27     /* automatic settings: RF 1, BB 2, DC 4 << register, IQ 64 */
#define ESP_STAT_PLL 28           /* capacitor code | lock window first << 9 | length << 18 */
#define ESP_STAT_COUNT 29

#define ESP_RADIO_OK 0
#define ESP_RADIO_PHY_FAILED 1
#define ESP_RADIO_PLL_FAILED 2
#define ESP_RADIO_PBUS_FAILED 3

/* ESP acquisition failure codes (ESP_STAT_STATUS). */
#define ESP_FAIL_OWNER 1          /* capture bank ownership changed unexpectedly */
#define ESP_FAIL_LATE_POLL 2      /* bank aged past its capacity while polling */
#define ESP_FAIL_LATE_SWITCH 3    /* bank aged past its capacity at the switch */
#define ESP_FAIL_END 4            /* end of the completed unit not found */
#define ESP_FAIL_START 5          /* start of the completed unit not where expected */
#define ESP_FAIL_LENGTH 6         /* unit length outside LINK_MIN/MAX_PAIRS */
#define ESP_FAIL_REUSE 7          /* next bank still being transmitted */
#define ESP_FAIL_LATE_PREPARE 8   /* next bank prepared after its switch point */
#define ESP_FAIL_RADIO 9          /* radio initialisation failed at boot */

/* FPGA operations. */
#define FPGA_REFERENCE 16 /* arg FPGA_REFERENCE_KEY: start 40 MHz ESP clock; 0: stop */
#define FPGA_ARM 17       /* open a new stream: reset the pipeline and start receiving */
#define FPGA_STOP 18      /* stop receiving; the stream ends with an END record */
#define FPGA_PHASE 19     /* arg: link sampling phase, 0..FPGA_PHASE_STEPS-1 (not while armed) */
#define FPGA_RELEASE 20   /* close the stream, stopping it first if it is still receiving */

/*
 * The FPGA carries one stream at a time. FPGA_ARM opens it and is answered
 * CTL_BUSY while a stream is open; the stream stays open after FPGA_STOP,
 * while its END record drains, until its owner closes it with FPGA_RELEASE.
 * A stream left open by an owner that died is closed the same way.
 */

/* The link is sampled at 240 MHz; its phase moves in steps of 1/56 of the
 * 1200 MHz MMCM VCO period (14.9 ps). The image starts at its built-in phase. */
#define FPGA_PHASE_STEPS 280

/*
 * The ESP32-S3 runs from the FPGA's 40 MHz reference. Only change it while
 * the ESP is held in reset (EN low). The pad is high impedance until enabled.
 */
#define FPGA_REFERENCE_KEY 40000

/*
 * FPGA statistics. Every statistic can be read at any time, including while
 * a stream runs; a value that spans two words (_LO, _HI) is read one word at
 * a time, so read it again if the high word changed in between.
 */
#define FPGA_STAT_FLAGS 0         /* FPGA_FLAG_* */
#define FPGA_STAT_PAIRS_LO 1      /* IQ pairs delivered to the encoder */
#define FPGA_STAT_PAIRS_HI 2
#define FPGA_STAT_STREAM_CRC 3    /* CRC32 of every accepted pair (LE32 words) */
#define FPGA_STAT_RECORDS 4       /* compressed records emitted, END excluded */
#define FPGA_STAT_UNITS0 5        /* link units received on lane 0 */
#define FPGA_STAT_UNITS1 6
#define FPGA_STAT_FRAMING0 7      /* units rejected: bad header or sequence (lost) */
#define FPGA_STAT_FRAMING1 8
#define FPGA_STAT_CHECKSUM0 9     /* units with a checksum mismatch */
#define FPGA_STAT_CHECKSUM1 10
#define FPGA_STAT_END_MARK0 11    /* units with a corrupt end marker */
#define FPGA_STAT_END_MARK1 12
#define FPGA_STAT_SAMPLE_OVERFLOW 13  /* 240 MHz capture FIFO overflows */
#define FPGA_STAT_REORDER_OVERFLOW0 14
#define FPGA_STAT_REORDER_OVERFLOW1 15
#define FPGA_STAT_LOST_UNITS 16   /* units missing from the stream (gaps) */
#define FPGA_STAT_RING_INPUT_OVERFLOW 17
#define FPGA_STAT_RING_OUTPUT_OVERFLOW 18
#define FPGA_STAT_RING_ERRORS 19
#define FPGA_STAT_RING_USED 20    /* current DDR ring occupancy, 16-byte beats */
#define FPGA_RING_BEATS 16777216  /* DDR ring capacity, 16-byte beats (256 MiB) */
#define FPGA_STAT_RING_PEAK 21    /* peak DDR ring occupancy, 16-byte beats */
#define FPGA_STAT_USB_UNDERRUNS 22
#define FPGA_STAT_USB_MAX_STALL 23 /* longest FT600 TXE-high stall, 100 MHz cycles */
#define FPGA_STAT_REORDER_PEAK0 24 /* peak reorder queue depth, pairs */
#define FPGA_STAT_REORDER_PEAK1 25
#define FPGA_STAT_PHASE 26        /* current link sampling phase */
#define FPGA_STAT_LOST_PAIRS_LO 27 /* IQ pairs in those gaps */
#define FPGA_STAT_LOST_PAIRS_HI 28
#define FPGA_STAT_DISCARDED_UNITS 29 /* units received with a misread header, dropped */
#define FPGA_STAT_COUNT 30

#define FPGA_FLAG_ARMED 0x01
#define FPGA_FLAG_DDR_READY 0x02
#define FPGA_FLAG_DESKEW_READY 0x04
#define FPGA_FLAG_PHASE_READY 0x08    /* sampling phase has reached its target */
#define FPGA_FLAG_REFERENCE_ON 0x10
#define FPGA_FLAG_STREAM_ENDED 0x20
#define FPGA_FLAG_STREAM_OPEN 0x40    /* opened by FPGA_ARM, not yet released */

#endif
