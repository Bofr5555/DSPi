/*
 * midi_control.h - USB MIDI control transport for DSPi (DSPI_USB_MIDI)
 *
 * An optional class-compliant USB MIDI function ("DSPi Control") that
 * carries the UART control frame (uart_control.h) inside SysEx, so hosts
 * without access to vendor control requests (CoreMIDI on iPhone) can reach
 * the full vendor command surface.  Wire format: midi_frame.h and
 * Documentation/Features/usb_midi_control_spec.md.
 *
 * Same design rule as the UART transport: nothing here blocks or
 * busy-waits.  TinyUSB's MIDI driver moves USB packets in and out of its
 * FIFOs from tud_task(); midi_ctrl_poll() parses, dispatches through
 * vendor_dispatch_get/set (CTRL_SOURCE_MIDI) and streams the response from
 * the main loop.
 */

#ifndef MIDI_CONTROL_H
#define MIDI_CONTROL_H

// Main-loop tick.  Never blocks; cheap no-op while the MIDI function is not
// configured by the host.
void midi_ctrl_poll(void);

#endif // MIDI_CONTROL_H
