//! Allwinner FEL library.

// #![forbid(missing_docs, warnings)]
#![deny(deprecated, improper_ctypes, non_shorthand_field_patterns, overflowing_literals,
    plugin_as_library, private_no_mangle_fns, private_no_mangle_statics, stable_features,
    unconditional_recursion, unknown_lints, unused, unused_allocation, unused_attributes,
    unused_comparisons, unused_features, unused_parens, while_true)]
#![warn(missing_docs, trivial_casts, trivial_numeric_casts, unused, unused_extern_crates,
    unused_import_braces, unused_qualifications, unused_results, variant_size_differences)]

#![allow(unused)]

// `error_chain!` can recurse deeply
#![recursion_limit = "1024"]

#[macro_use]
extern crate error_chain;
extern crate libusb;
extern crate byteorder;

use std::io;
use std::u32;
use std::ops::Deref;
use std::time::Duration;

use libusb::DeviceHandle;

/// FEL errors
#[allow(missing_docs)]
pub mod error {
    error_chain!{
        errors {
            /// USB response error.
            Response(expected: String, found: String) {
                description("invalid response")
                display("invalid response: expected '{}', found: {}", expected, found)
            }
            /// Unsupported device ID.
            UnsupportedDevId(id: u32) {
                description("unsupported device ID")
                display("unsupported device ID: {:#010x}", id)
            }
        }
        foreign_links {
            LibUsb(::libusb::Error);
            Io(::std::io::Error);
        }
    }
}

mod soc;
use soc::*;
use error::*;

/// USB timeout (in seconds).
const USB_TIMEOUT: u64 = 10;
/// `AW_USB_MAX_BULK_SEND` and the timeout constant `USB_TIMEOUT` are related. Both need to be
/// selected in a way that transferring the maximum chunk size with (SoC-specific) slow transfer
/// speed won't time out.
///
/// The *512 KiB* here are chosen based on the assumption that we want a 10 seconds timeout, and
/// "slow" transfers take place at approx. *64 KiB/sec*, so we can expect the maximum chunk being
/// transmitted within 8 seconds or less.
const AW_USB_MAX_BULK_SEND: usize = 512 * 1024; //512 KiB per bulk request

/// Allwinner FEL device USB vendor ID.
const AW_VENDOR_ID: u16 = 0x1f3a;
/// Allwinner FEL device USB product ID.
const AW_PRODUCT_ID: u16 = 0xefe8;
/// Allwinner USB read request.
const AW_USB_READ: u16 = 0x11;
/// Allwinner USB write request.
const AW_USB_WRITE: u16 = 0x12;

// Request types:
/// Allwinner FEL version request.
const AW_FEL_VERSION: u32 = 0x001;
/// Allwinner FEL write request.
const AW_FEL_1_WRITE: u32 = 0x101;
/// Allwinner FEL execution request.
const AW_FEL_1_EXEC: u32 = 0x102;
/// Allwinner FEL read request.
const AW_FEL_1_READ: u32 = 0x103;

// We don't want the scratch code/buffer to exceed a maximum size of `0x400` bytes (256 32-bit
// words) on `read_words()`/`write_words()` transfers. To guarantee this, we have to account for
// the amount of space the ARM code uses.
/// Word count of the `[read/write]_words()` scratch code.
const LCODE_ARM_RW_WORDS: usize = 12;
/// Word count of the `rmr_request` scratch code.
const LCODE_ARM_RMR_WORDS: usize = 15;
/// Code size in bytes.
const LCODE_ARM_RW_SIZE: usize = (LCODE_ARM_RW_WORDS << 2);
/// Maximum total words in buffer.
const LCODE_MAX_TOTAL: usize = 0x100;
/// Data words for read/write requests.
const LCODE_MAX_RW_WORDS: usize = (LCODE_MAX_TOTAL - LCODE_ARM_RW_WORDS);

/// Converts a reference to a `u32` slice to a reference to a `u8` slice.
///
/// The new slice will have a 4 times bigger length.
#[inline(always)]
fn u32_as_u8<'a>(src: &'a [u32]) -> &'a [u8] {
    unsafe { std::slice::from_raw_parts(src.as_ptr() as *mut u8, src.len() * 4) }
}

/// Converts a mutable reference to a `u32` slice to a mutable reference to a `u8` slice.
///
/// The new slice will have a 4 times bigger length.
#[inline(always)]
fn u32_as_u8_mut<'a>(src: &'a mut [u32]) -> &'a mut [u8] {
    unsafe { std::slice::from_raw_parts_mut(src.as_mut_ptr() as *mut u8, src.len() * 4) }
}

/// FEL device handle.
pub struct FelHandle<'h> {
    usb_handle: UsbHandle<'h>,
    soc_version: SocVersion,
    soc_info: SocInfo,
}

impl<'h> FelHandle<'h> {
    /// Gets the SoC information from the FEL device.
    ///
    /// Note: This is a no-op. The SoC information is acquired during initialization.
    pub fn get_soc_info(&self) -> &SocInfo {
        &self.soc_info
    }

    /// Gets the SoC version information from the FEL device.
    ///
    /// Note: This is a no-op. The SoC information is acquired during initialization.
    pub fn get_version_info(&self) -> &SocVersion {
        &self.soc_version
    }

    /// Reads the SID from the SoC if it has one.
    pub fn read_sid(&self) -> Result<Option<[u32; 4]>> {
        if let Some(sid_addr) = self.soc_info.get_sid_addr() {
            let mut sid = [0u32; 4];
            self.read_words(sid_addr, &mut sid)
                .chain_err(|| "unable to read SID registers")?;
            Ok(Some(sid))
        } else {
            Ok(None)
        }
    }

    /// Read words chunk from the FEL device memory.
    pub fn read_words(&self, offset: u32, words: &mut [u32]) -> Result<()> {
        assert!(u32::MAX - (words.len() * 4) as u32 > offset,
                "cannot read above {:#010x} – offset: {:#010x}, buffer length: {:#010x} words, \
                 words above offset: {:#010x}",
                u32::MAX,
                offset,
                words.len(),
                (u32::MAX - offset) / 4);
        for (i, chunk) in words.chunks_mut(LCODE_MAX_RW_WORDS).enumerate() {
            self.read_words_chunk(offset + (i * LCODE_ARM_RW_WORDS) as u32, chunk)
                .chain_err(|| "could not read word chunk from memory")?;
        }
        Ok(())
    }

    /// Write words chunk to the FEL device memory.
    pub fn write_words(&self, offset: u32, words: &[u32]) -> Result<()> {
        assert!(u32::MAX - (words.len() * 4) as u32 > offset,
                "cannot write above {:#010x} – offset: {:#010x}, buffer length: {:#010x} words, \
                 words above offset: {:#010x}",
                u32::MAX,
                offset,
                words.len(),
                (u32::MAX - offset) / 4);
        for (i, chunk) in words.chunks(LCODE_MAX_RW_WORDS).enumerate() {
            self.write_words_chunk(offset + (i * LCODE_ARM_RW_WORDS) as u32, chunk)
                .chain_err(|| "could not write word chunk to memory")?;
        }
        Ok(())
    }

    /// Read words chunk from the FEL device memory.
    fn read_words_chunk(&self, offset: u32, words: &mut [u32]) -> Result<()> {
        debug_assert!(words.len() <= LCODE_MAX_RW_WORDS,
                      "read_words requests cannot exceed {} words",
                      LCODE_MAX_RW_WORDS);
        let mut arm_code: [u32; LCODE_ARM_RW_WORDS] =
            [// `ldr   r0, [pc, #32] ; ldr r0,[read_addr]`
             0x_e5_9f_00_20_u32.to_le(),
             // `add   r1, pc, #36   ; adr r1, read_data`
             0x_e2_8f_10_24_u32.to_le(),
             // `ldr   r2, [pc, #28] ; ldr r2,[read_count]`
             0x_e5_9f_20_1c_u32.to_le(),
             // `cmp   r2, #LCODE_RW_MAX_WORDS`
             (0x_e3_52_00_00_u32 + LCODE_MAX_RW_WORDS as u32).to_le(),
             // `movgt r2, #LCODE_MAX_RW_WORDS`
             (0x_c3_a0_20_00_u32 + LCODE_MAX_RW_WORDS as u32).to_le(),
             // read_loop:
             // `subs  r2, r2, #1    ; r2 -= 1`
             0x_e2_52_20_01_u32.to_le(),
             // `bxmi  lr            ; return if (r2 < 0)`
             0x_41_2f_ff_1e_u32.to_le(),
             // `ldr   r3, [r0], #4  ; load and post-inc`
             0x_e4_90_30_04_u32.to_le(),
             // `str   r3, [r1], #4  ; store and post-inc`
             0x_e4_81_30_04_u32.to_le(),
             // `b     read_loop`
             0x_ea_ff_ff_fa_u32.to_le(),
             offset.to_le(),
             (words.len() as u32).to_le()];
        // read_data (buffer) follows, i.e. values go here.

        /// scratch buffer setup: transfers ARM code, including addr and count
        self.usb_handle
            .fel_write(self.soc_info.get_scratch_addr(), u32_as_u8(&arm_code))
            .chain_err(|| "unable to write ARM code to scratch address")?;
        // execute code, read back the result
        self.usb_handle
            .fel_execute(self.soc_info.get_scratch_addr())
            .chain_err(|| "unable to execute ARM code")?;
        self.usb_handle
            .fel_read(self.soc_info.get_scratch_addr() + LCODE_ARM_RW_SIZE as u32,
                      u32_as_u8_mut(words))
            .chain_err(|| "unable to read generated buffer")?;

        if cfg!(not(target_endian = "little")) {
            for word in words.iter_mut() {
                *word = word.to_le();
            }
        }

        Ok(())
    }

    /// Write words chunk to the FEL device memory.
    fn write_words_chunk(&self, offset: u32, words: &[u32]) -> Result<()> {
        debug_assert!(words.len() <= LCODE_MAX_RW_WORDS,
                      "write_words requests cannot exceed {} words",
                      LCODE_MAX_RW_WORDS);

        let mut arm_code = Vec::with_capacity(LCODE_ARM_RW_WORDS + words.len());
        arm_code.extend_from_slice(&[// `ldr   r0, [pc, #32] ; ldr r0,[write_addr]`
                                     0x_e5_9f_00_20_u32.to_le(),
                                     // `add   r1, pc, #36   ; adr r1, write_data`
                                     0x_e2_8f_10_24_u32.to_le(),
                                     // `ldr   r2, [pc, #28] ; ldr r2,[write_count]`
                                     0x_e5_9f_20_1c_u32.to_le(),
                                     // `cmp   r2, #LCODE_MAX_RW_WORDS`
                                     (0x_e3_52_00_00_u32 + LCODE_MAX_RW_WORDS as u32).to_le(),
                                     // `movgt r2, #LCODE_MAX_RW_WORDS`
                                     (0x_c3_a0_20_00_u32 + LCODE_MAX_RW_WORDS as u32).to_le(),
                                     // write_loop:
                                     // `subs  r2, r2, #1    ; r2 -= 1`
                                     0x_e2_52_20_01_u32.to_le(),
                                     // `bxmi  lr            ; return if (r2 < 0)`
                                     0x_41_2f_ff_1e_u32.to_le(),
                                     // `ldr   r3, [r1], #4  ; load and post-inc`
                                     0x_e4_91_30_04_u32.to_le(),
                                     // `str   r3, [r0], #4  ; store and post-inc`
                                     0x_e4_80_30_04_u32.to_le(),
                                     // `b     write_loop`
                                     0x_ea_ff_ff_fa_u32.to_le(),
                                     offset.to_le(),
                                     (words.len() as u32).to_le()]);
        // write_data (buffer) follows, i.e. values go here.
        // copy values from source buffer
        for word in words.iter() {
            arm_code.push(word.to_le());
        }

        /// scratch buffer setup: transfers ARM code and data
        self.usb_handle
            .fel_write(self.soc_info.get_scratch_addr(), u32_as_u8(&arm_code))
            .chain_err(|| "unable to write ARM code to scratch address")?;
        // execute code
        self.usb_handle
            .fel_execute(self.soc_info.get_scratch_addr())
            .chain_err(|| "unable to execute ARM code")?;

        Ok(())
    }

    /// Perform FEL RMR request.
    ///
    /// This function stores a given entry point to the `RVBAR` address for *CPU0*, and then writes
    /// the Reset Management Register to request a warm boot.
    /// It is useful with some *AArch64* transitions, e.g. when passing control to ARM Trusted
    /// Firmware (ATF) during the boot process of Pine64.
    ///
    /// The code was inspired by
    /// [U-boot](https://github.com/apritzel/u-boot/commit/fda6bd1bf285c44f30ea15c7e6231bf53c31d4a8)
    pub fn rmr_request(&self, entry_point: u32, aarch64: bool) -> Result<()> {
        if let Some(rvbar_reg) = self.soc_info.get_rvbar_reg() {
            let rmr_mode: u32 = (1 << 1) | (if aarch64 { 1 } else { 0 });

            let arm_code: [u32; LCODE_ARM_RMR_WORDS] = [// ldr    r0, [rvbar_reg]
                                                        0x_e5_9f_00_28_u32.to_le(),
                                                        // ldr    r1, [entry_point]
                                                        0x_e5_9f_10_28_u32.to_le(),
                                                        // str    r1, [r0]
                                                        0x_e5_80_10_00_u32.to_le(),
                                                        // dsb    sy
                                                        0x_f5_7f_f0_4f_u32.to_le(),
                                                        // isb    sy
                                                        0x_f5_7f_f0_6f_u32.to_le(),

                                                        // ldr    r1, [rmr_mode]
                                                        0x_e5_9f_10_1c_u32.to_le(),
                                                        // mrc    15, 0, r0, cr12, cr0, {2}
                                                        0x_ee_1c_0f_50_u32.to_le(),
                                                        // orr    r0, r0, r1
                                                        0x_e1_80_00_01_u32.to_le(),
                                                        // mcr    15, 0, r0, cr12, cr0, {2}
                                                        0x_ee_0c_0f_50_u32.to_le(),
                                                        // isb    sy
                                                        0x_f5_7f_f0_6f_u32.to_le(),
                                                        // loop:  wfi
                                                        0x_e3_20_f0_03_u32.to_le(),
                                                        // b      <loop>
                                                        0x_ea_ff_ff_fd_u32.to_le(),

                                                        rvbar_reg.to_le(),
                                                        entry_point.to_le(),
                                                        rmr_mode.to_le()];

            self.usb_handle
                .fel_write(self.soc_info.get_scratch_addr(), u32_as_u8(&arm_code))
                .chain_err(|| "unable to write ARM code to scratch address")?;
            self.usb_handle
                .fel_execute(self.soc_info.get_scratch_addr())
                .chain_err(|| "unable to execute ARM code")?;
            Ok(())
        } else {
            bail!("can't issue RMR request, the SoC ({}) does not support RVBAR or it's unknown",
                  self.soc_info.get_name());
        }
    }
}

impl<'h> Deref for FelHandle<'h> {
    type Target = UsbHandle<'h>;

    fn deref(&self) -> &UsbHandle<'h> {
        &self.usb_handle
    }
}

/// USB device handle.
pub struct UsbHandle<'h> {
    device_handle: DeviceHandle<'h>,
    endpoint_in: u8,
    endpoint_out: u8,
    iface_detached: bool,
}

impl<'h> UsbHandle<'h> {
    /// Creates a USB handle from the given device.
    fn from_device(device: libusb::Device<'h>) -> Result<UsbHandle<'h>> {
        let mut handle = UsbHandle {
            device_handle: device.open().chain_err(|| "unable to open device")?,
            endpoint_in: 0,
            endpoint_out: 0,
            iface_detached: false,
        };
        handle.claim(device)?;
        Ok(handle)
    }

    /// Claim the given device.
    fn claim(&mut self, device: libusb::Device<'h>) -> Result<()> {
        if let Err(e) = self.device_handle.claim_interface(0) {
            if cfg!(target_os = "linux") {
                self.device_handle
                    .detach_kernel_driver(0)
                    .chain_err(|| "unable to detach kernel driver")?;
                self.iface_detached = true;
                self.device_handle
                    .claim_interface(0)
                    .chain_err(|| "unable to claim device interface")?;
            } else {
                return Err(e).chain_err(|| "unable to claim device interface");
            }
        }
        self.get_endpoints(device).chain_err(|| "unable to get device endpoints")?;
        Ok(())
    }

    /// Updates the endpoints of the USB handle.
    fn get_endpoints(&mut self, device: libusb::Device<'h>) -> Result<()> {
        use libusb::{TransferType, Direction};

        let config_descriptor = device.active_config_descriptor()
            .chain_err(|| "unable to get active config descriptor")?;
        for interface in config_descriptor.interfaces() {
            for descriptor in interface.descriptors() {
                for endpoint in descriptor.endpoint_descriptors() {
                    if let TransferType::Bulk = endpoint.transfer_type() {
                        match endpoint.direction() {
                            Direction::In => self.endpoint_in = endpoint.address(),
                            Direction::Out => self.endpoint_out = endpoint.address(),
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// Fill memory at the given offset.
    ///
    /// It will fill `num_size` bytes with the given byte.
    pub fn fel_fill(&self, offset: u32, num_bytes: u32, byte: u8) -> Result<()> {
        assert!(u32::MAX - num_bytes > offset,
                "cannot write above {:#010x} – offset: {:#010x}, num_bytes: {:#010x}, bytes \
                 above offset: {:#010x}",
                u32::MAX,
                offset,
                num_bytes,
                u32::MAX - offset);
        let buf = vec![byte; num_bytes as usize];
        let _ = self.fel_write(offset, &buf)
            .chain_err(|| "unable to write filling buffer to device memory")?;
        Ok(())
    }

    /// Reads from the FEL device memory at the given offset.
    ///
    /// It will fill all the given buffer, but it **will panic** if the buffer overflows total
    /// memory address space.
    pub fn fel_read(&self, offset: u32, buf: &mut [u8]) -> Result<()> {
        assert!(u32::MAX - buf.len() as u32 > offset,
                "cannot read above {:#010x} – offset: {:#010x}, buffer length: {:#010x}, bytes \
                 above offset: {:#010x}",
                u32::MAX,
                offset,
                buf.len(),
                u32::MAX - offset);
        self.send_fel_request(AW_FEL_1_READ, offset, buf.len() as u32)
            .chain_err(|| "unable to send AW_FEL_1_READ FEL request")?;
        self.usb_read(buf).chain_err(|| "unable read data from USB")?;
        self.read_fel_status().chain_err(|| "unable to read FEL status")?;
        Ok(())
    }

    /// Writes to the FEL device memory at the given offset.
    ///
    /// It **will panic** if the buffer overflows total memory address space.
    pub fn fel_write(&self, offset: u32, buf: &[u8]) -> Result<()> {
        assert!(u32::MAX - buf.len() as u32 > offset,
                "cannot write above {:#010x} – offset: {:#010x}, buffer length: {:#010x}, bytes \
                 above offset: {:#010x}",
                u32::MAX,
                offset,
                buf.len(),
                u32::MAX - offset);
        self.send_fel_request(AW_FEL_1_WRITE, offset, buf.len() as u32)
            .chain_err(|| "unable to send AW_FEL_1_WRITER FEL request")?;
        self.usb_write(buf).chain_err(|| "unable write data to USB")?;
        self.read_fel_status().chain_err(|| "unable to read FEL status")?;
        Ok(())
    }

    /// Makes the FEL device execute the code at the given address.
    pub fn fel_execute(&self, offset: u32) -> Result<()> {
        self.send_fel_request(AW_FEL_1_EXEC, offset, 0)
            .chain_err(|| "unable to send AW_FEL_1_EXEC FEL request")?;
        self.read_fel_status().chain_err(|| "unable to read FEL status")?;
        Ok(())
    }

    /// Gets the SoC version information.
    fn get_fel_version(&self) -> Result<SocVersion> {
        self.send_fel_request(AW_FEL_VERSION, 0, 0)
            .chain_err(|| "unable to send AW_FEL_VERSION FEL request")?;
        let mut buf = [0u8; 32];
        self.usb_read(&mut buf).chain_err(|| "unable to read version from USB")?;
        self.read_fel_status().chain_err(|| "unable to read FEL status")?;

        Ok(SocVersion::from_bytes(buf))
    }

    /// Reads the FEL status.
    fn read_fel_status(&self) -> Result<()> {
        let mut buf = [0u8; 8];
        self.usb_read(&mut buf).chain_err(|| "unable to read from USB")?;
        Ok(())
    }

    /// Sends a FEL request.
    fn send_fel_request(&self, req_type: u32, addr: u32, len: u32) -> Result<()> {
        use byteorder::{LittleEndian, ByteOrder};

        let mut request = [0u8; 16];
        LittleEndian::write_u32(&mut request[..4], req_type);
        LittleEndian::write_u32(&mut request[4..8], addr);
        LittleEndian::write_u32(&mut request[8..12], len);
        // 4 byte padding.

        self.usb_write(&request).chain_err(|| "unable to write to USB")?;
        Ok(())
    }

    /// Reads the given data from the USB.
    fn usb_read(&self, data: &mut [u8]) -> Result<()> {
        self.send_usb_request(AW_USB_READ, data.len() as u32)
            .chain_err(|| "unable to send AW_USB_READ USB request")?;
        UsbHandle::usb_bulk_recv(&self.device_handle, self.endpoint_in, data)
                .chain_err(|| "unable to receive data in bulk from USB after AW_USB_READ USB \
                               request")?;
        self.read_usb_response().chain_err(|| "unable to read response from USB")?;
        Ok(())
    }

    /// Writes the given data to the USB.
    fn usb_write(&self, data: &[u8]) -> Result<()> {
        self.send_usb_request(AW_USB_WRITE, data.len() as u32)
            .chain_err(|| "unable to send AW_USB_WRITE USB request")?;
        UsbHandle::usb_bulk_send(&self.device_handle, self.endpoint_out, data)
                .chain_err(|| "unable to send data in bulk to USB after AW_USB_WRITE USB request")?;
        self.read_usb_response().chain_err(|| "unable to read response from USB")?;
        Ok(())
    }

    /// Sends the request type USB request.
    fn send_usb_request(&self, req_type: u16, len: u32) -> Result<()> {
        use byteorder::{LittleEndian, ByteOrder};

        let mut request = [0u8; 32];
        request[..4].clone_from_slice(b"AWUC");
        LittleEndian::write_u32(&mut request[8..12], len);
        LittleEndian::write_u32(&mut request[12..16], 0x0C000000); // Unknown data
        LittleEndian::write_u16(&mut request[16..18], req_type);
        LittleEndian::write_u32(&mut request[18..22], len); // Length is repeated
        // Bytes from 22 to 32 are padding

        UsbHandle::usb_bulk_send(&self.device_handle, self.endpoint_out, &request)
                .chain_err(|| "unable to send data in bulk to USB")?;
        Ok(())
    }

    /// Reads and discards the USB response. It will check if the response is correct.
    fn read_usb_response(&self) -> Result<()> {
        let mut buf = [0u8; 13];
        UsbHandle::usb_bulk_recv(&self.device_handle, self.endpoint_in, &mut buf)
                .chain_err(|| "unable to receive data in bulk from USB")?;
        if &buf[..4] != b"AWUS" {
            Err(Error::from_kind(ErrorKind::Response("AWUS[...]".to_owned(),
                                                     format!("{}[...]",
                                                             String::from_utf8_lossy(&buf[..4])))))
        } else {
            Ok(())
        }
    }

    /// Sends data to the USB in bulk.
    ///
    /// It will divide the data in chunks.
    fn usb_bulk_send(device_handle: &DeviceHandle, endpoint: u8, data: &[u8]) -> Result<()> {
        let mut sent = 0;
        while sent < data.len() {
            let slice = if data.len() - sent < AW_USB_MAX_BULK_SEND {
                &data[sent..]
            } else {
                &data[sent..sent + AW_USB_MAX_BULK_SEND]
            };
            sent += device_handle.write_bulk(endpoint, slice, Duration::from_secs(USB_TIMEOUT))
                .chain_err(|| {
                    format!("unable to send data chunk to USB, sent {} bytes: {:?}",
                            sent,
                            &data[..sent])
                })?;
        }

        Ok(())
    }

    /// Receives data from the USB in bulk.
    ///
    /// It will divide the data in chunks.
    fn usb_bulk_recv(device_handle: &DeviceHandle, endpoint: u8, data: &mut [u8]) -> Result<()> {
        let mut received = 0;
        while received < data.len() {
            let slice = if data.len() - received < AW_USB_MAX_BULK_SEND {
                &mut data[received..]
            } else {
                &mut data[received..received + AW_USB_MAX_BULK_SEND]
            };
            received += device_handle.read_bulk(endpoint, slice, Duration::from_secs(USB_TIMEOUT))
                .chain_err(|| {
                    format!("unable to read data chunk from USB, received {} bytes",
                            received)
                })?;
        }
        Ok(())
    }
}

impl<'h> Drop for UsbHandle<'h> {
    fn drop(&mut self) {
        use std::io::Write;
        use std::error::Error;
        if let Err(e) = self.device_handle.release_interface(0) {
            io::stderr()
                .write_all(format!("error releasing device handle interface: {:?} ({})",
                                   e,
                                   e.description())
                    .as_bytes())
                .unwrap();
        }
        if cfg!(target_os = "linux") && self.iface_detached {
            if let Err(e) = self.device_handle.attach_kernel_driver(0) {
                io::stderr()
                    .write_all(format!("error attaching kernel driver: {:?} ({})",
                                       e,
                                       e.description())
                        .as_bytes())
                    .unwrap();
            }
        }
    }
}

/// Allwinner FEL devices context.
pub struct Fel {
    context: libusb::Context,
}

impl Fel {
    /// Creates a new Fel object.
    pub fn new() -> Result<Fel> {
        Ok(Fel { context: libusb::Context::new().chain_err(|| "unable to create libUSB context")? })
    }

    /// Get the device from the given bus and address, if it exists and is a FEL device.
    pub fn get_device(&self, bus: u8, address: u8) -> Result<Option<FelHandle>> {
        for device in self.context.devices().chain_err(|| "unable to list USB devices")?.iter() {
            if device.bus_number() == bus && device.address() == address {
                let device_descriptor = device.device_descriptor()
                    .chain_err(|| "unable to get USB device descriptor")?;
                if device_descriptor.vendor_id() == AW_VENDOR_ID &&
                   device_descriptor.product_id() == AW_PRODUCT_ID {
                    let usb_handle = UsbHandle::from_device(device)?;
                    let soc_version = usb_handle.get_fel_version()?;
                    return if let Some(soc_info) = SocInfo::from_version(&soc_version) {
                        Ok(Some(FelHandle {
                            usb_handle: usb_handle,
                            soc_version: soc_version,
                            soc_info: soc_info,
                        }))
                    } else {
                        Err(Error::from_kind(ErrorKind::UnsupportedDevId(soc_version.get_id())))
                    };
                } else {
                    return Ok(None);
                }
            }
        }
        Ok(None)
    }

    /// Creates a list of Allwinner devices in Fel mode.
    pub fn list_devices(&self) -> Result<Vec<FelHandle>> {
        let mut result = Vec::new();
        for device in self.context.devices().chain_err(|| "unable to list USB devices")?.iter() {
            let device_descriptor = device.device_descriptor()
                .chain_err(|| "unable to get USB device descriptor")?;
            if device_descriptor.vendor_id() == AW_VENDOR_ID &&
               device_descriptor.product_id() == AW_PRODUCT_ID {
                let usb_handle = UsbHandle::from_device(device)?;
                let soc_version = usb_handle.get_fel_version()?;
                if let Some(soc_info) = SocInfo::from_version(&soc_version) {
                    result.push(FelHandle {
                        usb_handle: usb_handle,
                        soc_version: soc_version,
                        soc_info: soc_info,
                    })
                } else {
                    return Err(Error::from_kind(ErrorKind::UnsupportedDevId(soc_version.get_id())));
                }
            }
        }
        Ok(result)
    }
}
