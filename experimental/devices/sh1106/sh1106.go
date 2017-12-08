// Copyright 2016 The Periph Authors. All rights reserved.
// Use of this source code is governed under the Apache License, Version 2.0
// that can be found in the LICENSE file.

// Package sh1106 controls a 128x64 monochrome OLED display via a SH1106
// controller.
//
// The driver does differential updates: it only sends modified pixels for the
// smallest rectangle, to economize bus bandwidth. This is especially important
// when using I²C as the bus default speed (often 100kHz) is slow enough to
// saturate the bus at less than 10 frames per second.
//
// The SH1106 is a write-only device. It can be driven on either I²C or SPI
// with 4 wires. Changing between protocol is likely done through resistor
// soldering, for boards that support both.
//
// Some boards expose a RES / Reset pin. If present, it must be normally be
// High. When set to Low (Ground), it enables the reset circuitry. It can be
// used externally to this driver, if used, the driver must be reinstantiated.
//
// Datasheets
//
// Product page:
// http://www.solomon-systech.com/en/product/display-ic/oled-driver-controller/sh1106/
//
// https://cdn-shop.adafruit.com/datasheets/SH1106.pdf
//
// "DM-OLED096-624": https://drive.google.com/file/d/0B5lkVYnewKTGaEVENlYwbDkxSGM/view
//
// "sh1106": https://drive.google.com/file/d/0B5lkVYnewKTGYzhyWWp0clBMR1E/view
package sh1106

// Some have SPI enabled;
// https://hallard.me/adafruit-oled-display-driver-for-pi/
// https://learn.adafruit.com/sh1106-oled-displays-with-raspberry-pi-and-beaglebone-black?view=all

import (
	"bytes"
	"errors"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"log"

	"periph.io/x/periph/conn"
	"periph.io/x/periph/conn/gpio"
	"periph.io/x/periph/conn/i2c"
	"periph.io/x/periph/conn/spi"
	"periph.io/x/periph/devices"
	"periph.io/x/periph/experimental/devices/sh1106/image1bit"
)

// Dev is an open handle to the display controller.
type Dev struct {
	// Communication
	c   conn.Conn
	dc  gpio.PinOut
	spi bool

	// Display size controlled by the SH1106.
	rect image.Rectangle

	// Mutable
	// See page 25 for the GDDRAM pages structure.
	// Narrow screen will waste the end of each page.
	// Short screen will ignore the lower pages.
	// There is 8 pages, each covering an horizontal band of 8 pixels high (1
	// byte) for 128 bytes.
	// 8*128 = 1024 bytes total for 128x64 display.
	buffer []byte
	// next is lazy initialized on first Draw(). Write() skips this buffer.
	next               *image1bit.VerticalLSB
	startPage, endPage int
	startCol, endCol   int
	err                error
}

// NewSPI returns a Dev object that communicates over SPI to a SH1106 display
// controller.
//
// If rotated is true, turns the display by 180°
//
// The SH1106 can operate at up to 3.3Mhz, which is much higher than I²C. This
// permits higher refresh rates.
//
// Wiring
//
// Connect SDA to MOSI, SCK to SCLK, CS to CS.
//
// In 3-wire SPI mode, pass nil for 'dc'. In 4-wire SPI mode, pass a GPIO pin
// to use.
//
// The RES (reset) pin can be used outside of this driver but is not supported
// natively. In case of external reset via the RES pin, this device drive must
// be reinstantiated.
func NewSPI(p spi.Port, dc gpio.PinOut, w, h int, rotated bool) (*Dev, error) {
	if dc == gpio.INVALID {
		return nil, errors.New("sh1106: use nil for dc to use 3-wire mode, do not use gpio.INVALID")
	}
	bits := 8
	if dc == nil {
		// 3-wire SPI uses 9 bits per word.
		bits = 9
	} else if err := dc.Out(gpio.Low); err != nil {
		return nil, err
	}
	c, err := p.Connect(3300000, spi.Mode0, bits)
	if err != nil {
		return nil, err
	}
	return newDev(c, w, h, rotated, true, dc)
}

// NewI2C returns a Dev object that communicates over I²C to a SH1106 display
// controller.
//
// If rotated, turns the display by 180°
func NewI2C(i i2c.Bus, w, h int, rotated bool) (*Dev, error) {
	// Maximum clock speed is 1/2.5µs = 400KHz.
	return newDev(&i2c.Dev{Bus: i, Addr: 0x3C}, w, h, rotated, false, nil)
}

// newDev is the common initialization code that is independent of the
// communication protocol (I²C or SPI) being used.
func newDev(c conn.Conn, w, h int, rotated, usingSPI bool, dc gpio.PinOut) (*Dev, error) {
	if w < 8 || w > 128 || w&7 != 0 {
		return nil, fmt.Errorf("sh1106: invalid width %d", w)
	}
	if h < 8 || h > 64 || h&7 != 0 {
		return nil, fmt.Errorf("sh1106: invalid height %d", h)
	}

	nbPages := h / 8
	pageSize := w
	d := &Dev{
		c:         c,
		spi:       usingSPI,
		dc:        dc,
		rect:      image.Rect(0, 0, int(w), int(h)),
		buffer:    make([]byte, nbPages*pageSize),
		startPage: 0,
		endPage:   nbPages,
		startCol:  0,
		endCol:    w,
	}
	if err := d.sendCommand(getInitCmd()); err != nil {
		return nil, err
	}
	return d, nil
}

func getInitCmd() []byte {
	return []byte{
		0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00,
		(0x40 | 0x00), 0x8D,
		0x14, // 0x10
		0x20, 0x00, (0xA0 | 0x01), 0xC8, 0xDA, 0x12, 0x81,
		0xCF, //0x9F
		0xD9, // 0xd9
		0xF1, // 0x22
		0xDB, 0x40, 0xA4, 0xA6,
		0x8D, 0x14, 0xAF,
	}
}

func (d *Dev) String() string {
	if d.spi {
		return fmt.Sprintf("sh1106.Dev{%s, %s, %s}", d.c, d.dc, d.rect.Max)
	}
	return fmt.Sprintf("sh1106.Dev{%s, %s}", d.c, d.rect.Max)
}

// ColorModel implements devices.Display.
//
// It is a one bit color model, as implemented by image1bit.Bit.
func (d *Dev) ColorModel() color.Model {
	return image1bit.BitModel
}

// Bounds implements devices.Display. Min is guaranteed to be {0, 0}.
func (d *Dev) Bounds() image.Rectangle {
	return d.rect
}

// Draw implements devices.Display.
//
// It draws synchronously, once this function returns, the display is updated.
// It means that on slow bus  (I²C), it may be preferable to defer Draw() calls
// to a background goroutine.
//
// It discards any failure.
func (d *Dev) Draw(r image.Rectangle, src image.Image, sp image.Point) {
	var next []byte
	if img, ok := src.(*image1bit.VerticalLSB); ok && r == d.Bounds() && src.Bounds() == d.rect && sp.X == 0 && sp.Y == 0 {
		// Exact size, full frame, image1bit encoding: fast path!
		next = img.Pix
	} else {
		// Double buffering.
		if d.next == nil {
			d.next = image1bit.NewVerticalLSB(d.rect)
		}
		next = d.next.Pix
		draw.Src.Draw(d.next, r, src, sp)
	}
	d.err = d.drawInternal(next)
}

// Err returns the last error that occurred
func (d *Dev) Err() error {
	return d.err
}

// Write writes a buffer of pixels to the display.
//
// The format is unsual as each byte represent 8 vertical pixels at a time. The
// format is horizontal bands of 8 pixels high.
//
// This function accepts the content of image1bit.VerticalLSB.Pix.
func (d *Dev) Write(pixels []byte) (int, error) {
	if len(pixels) != len(d.buffer) {
		return 0, fmt.Errorf("sh1106: invalid pixel stream length; expected %d bytes, got %d bytes", len(d.buffer), len(pixels))
	}
	// Write() skips d.next so it saves 1kb of RAM.
	if err := d.drawInternal(pixels); err != nil {
		return 0, err
	}
	return len(pixels), nil
}

// SetContrast changes the screen contrast.
//
// Note: values other than 0xff do not seem useful...
func (d *Dev) SetContrast(level byte) error {
	return d.sendCommand([]byte{0x81, level})
}

// Halt turns off the display.
//
// Sending any other command afterward reenables the display.
func (d *Dev) Halt() error {
	return d.sendCommand([]byte{0xAE})
}

// Invert the display (black on white vs white on black).
func (d *Dev) Invert(blackOnWhite bool) error {
	b := []byte{0xA6}
	if blackOnWhite {
		b[0] = 0xA7
	}
	return d.sendCommand(b)
}

//

func (d *Dev) calculateSubset(next []byte) (int, int, int, int, bool) {
	w := d.rect.Dx()
	h := d.rect.Dy()
	startPage := 0
	endPage := h / 8
	startCol := 0
	endCol := w
	// Calculate the smallest square that need to be sent.
	pageSize := w

	// Top.
	for ; startPage < endPage; startPage++ {
		x := pageSize * startPage
		y := pageSize * (startPage + 1)
		if !bytes.Equal(d.buffer[x:y], next[x:y]) {
			break
		}
	}
	// Bottom.
	for ; endPage > startPage; endPage-- {
		x := pageSize * (endPage - 1)
		y := pageSize * endPage
		if !bytes.Equal(d.buffer[x:y], next[x:y]) {
			break
		}
	}
	if startPage == endPage {
		// Early exit, the image is exactly the same.
		return 0, 0, 0, 0, true
	}
	// TODO(maruel): This currently corrupts the screen. Likely a small error
	// in the way the commands are sent.
	/*
			// Left.
			for ; startCol < endCol; startCol++ {
				for i := startPage; i < endPage; i++ {
					x := i*pageSize + startCol
					if d.buffer[x] != next[x] {
						goto breakLeft
					}
				}
			}
		breakLeft:
			// Right.
			for ; endCol > startCol; endCol-- {
				for i := startPage; i < endPage; i++ {
					x := i*pageSize + endCol - 1
					if d.buffer[x] != next[x] {
						goto breakRight
					}
				}
			}
		breakRight:
	*/
	return startPage, endPage, startCol, endCol, false
}

// drawInternal sends image data to the controller.
func (d *Dev) drawInternal(next []byte) error {
	startPage, endPage, startCol, endCol, skip := d.calculateSubset(next)
	if skip {
		return nil
	}
	copy(d.buffer, next)

	log.Println("disp:", d.startPage, d.endPage, d.startCol, d.endCol)
	log.Println("img:", startPage, endPage, startCol, endCol)

	if d.startPage != startPage || d.endPage != endPage || d.startCol != startCol || d.endCol != endCol {
		d.startPage = startPage
		d.endPage = endPage
		d.startCol = startCol
		d.endCol = endCol
		cmd := []byte{
			0x21, uint8(d.startCol), uint8(d.endCol - 1), // Set column address (Width)
			0x22, uint8(d.startPage), uint8(d.endPage - 1), // Set page address (Pages)
		}
		if err := d.sendCommand(cmd); err != nil {
			return err
		}
	}

	// Write the subset of the data as needed.
	pageSize := d.rect.Dx()
	return d.sendData(d.buffer[startPage*pageSize+startCol : (endPage-1)*pageSize+endCol])
}

func (d *Dev) sendData(c []byte) error {
	if d.spi {
		// this is in SPI mode
		// 4-wire SPI.
		if err := d.dc.Out(gpio.High); err != nil {
			return err
		}
		return d.c.Tx(c, nil)
	}
	// this is in I2C mode
	return d.c.Tx(append([]byte{i2cData}, c...), nil)
}

func (d *Dev) sendCommand(c []byte) error {
	if d.spi {
		// this is in SPI mode
		if d.dc == nil {
			// 3-wire SPI.
			return errors.New("sh1106: 3-wire SPI mode is not yet implemented")
		}
		// 4-wire SPI.
		if err := d.dc.Out(gpio.Low); err != nil {
			return err
		}
		return d.c.Tx(c, nil)
	}
	// this is in I2C mode
	return d.c.Tx(append([]byte{i2cCmd}, c...), nil)
}

const (
	i2cCmd  = 0x00 // I²C transaction has stream of command bytes
	i2cData = 0x40 // I²C transaction has stream of data bytes
)

var _ devices.Display = &Dev{}
var _ devices.Device = &Dev{}