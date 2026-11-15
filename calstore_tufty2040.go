//go:build tufty2040

package main

import (
	"errors"
	"machine"
)

// flashCalStore keeps the calibration record (app.CalStore) in the last
// 4 KB sector of the flash TinyGo leaves the firmware for data
// (machine.Flash: from the end of the image to the end of the linked
// 1020 KB). The data region starts right after the image, so its end is
// clear of a firmware that grows; a UF2 write (picoflash, the bootloader)
// touches only its image's own sectors.
//
// The region's start is not sector-aligned (the end of .data), and
// EraseBlocks counts blocks from that start: block k's address is
// start + k·4 KB. The record's sector is addressed absolutely instead (the
// region's end is aligned), and erased through the block whose address
// falls inside it (the chip erases the sector that holds the address).
type flashCalStore struct{}

var calPage [256]byte // one write block: WriteAt pads (appends) anything shorter

var errCalNoRoom = errors.New("no flash left for the calibration")

// calSector is the record's offset into the data region, and the block
// EraseBlocks needs to erase its sector.
func calSector() (off, block int64, err error) {
	es := machine.Flash.EraseBlockSize()
	size := machine.Flash.Size()
	if size < 2*es {
		return 0, 0, errCalNoRoom
	}
	off = size - es // the last sector: the region's end is sector-aligned
	block = (off + es - 1) / es
	return off, block, nil
}

func (flashCalStore) Read(p []byte) error {
	off, _, err := calSector()
	if err != nil {
		return err
	}
	_, err = machine.Flash.ReadAt(p, off)
	return err
}

func (flashCalStore) Write(p []byte) error {
	off, block, err := calSector()
	if err != nil {
		return err
	}
	if err := machine.Flash.EraseBlocks(block, 1); err != nil {
		return err
	}
	for i := range calPage {
		calPage[i] = 0xff
	}
	copy(calPage[:], p)
	_, err = machine.Flash.WriteAt(calPage[:], off)
	return err
}

func (flashCalStore) Erase() error {
	_, block, err := calSector()
	if err != nil {
		return err
	}
	return machine.Flash.EraseBlocks(block, 1)
}
