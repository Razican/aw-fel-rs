use std::time::Duration;
use std::{fmt, thread};

use byteorder::{BigEndian, ByteOrder, LittleEndian};
use failure::{Error, ResultExt};

mod fel2spl_thunk;
use self::fel2spl_thunk::*;

use super::{u32_as_u8, FelError, FelHandle, SPL_LEN_LIMIT};

/// *U-boot* image name length.
const UBOOT_IH_NMLEN: u32 = 32;
/// *U-Boot* image magic number.
const UBOOT_IH_MAGIC: u32 = 0x2705_1956;
/// ARM architechture constant in *U-Boot*.
const UBOOT_IH_ARCH_ARM: u8 = 0x02;
/// Offset of name field.
const UBOOT_HEADER_NAME_OFFSET: u32 = 32;
/// Header of the *U-Boot* header.
const UBOOT_HEADER_SIZE: u32 = (UBOOT_HEADER_NAME_OFFSET + UBOOT_IH_NMLEN);

/// *U-boot* image type.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum UbootImageType {
    /// Firmware image.
    Firmware,
    /// Script file.
    Script,
}

impl UbootImageType {
    fn from_byte(byte: u8) -> Result<Self, Error> {
        match byte {
            0x05 => Ok(UbootImageType::Firmware),
            0x06 => Ok(UbootImageType::Script),
            t => bail!("Unknown image type ({:#04x})", t),
        }
    }
}

impl fmt::Display for UbootImageType {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let text = match *self {
            UbootImageType::Firmware => "firmware",
            UbootImageType::Script => "script",
        };
        write!(f, "{}", text)
    }
}

impl<'h> FelHandle<'h> {
    /// This function tests a given buffer for a valid *U-Boot* image. Upon success, the image data
    /// gets transferred to the default memory address stored within the image header; and the
    /// function returns the *U-Boot* entry point (offset) and size values, in that order.
    pub fn write_uboot_image(&self, uboot: &[u8]) -> Result<(u32, u32), Error> {
        if uboot.len() <= UBOOT_HEADER_SIZE as usize {
            bail!("insufficient U-boot size");
        }

        let image_type = get_image_type(uboot)?;
        if image_type != UbootImageType::Firmware {
            bail!("expected firmware image, got {} image", image_type)
        }

        let data_size = BigEndian::read_u32(&uboot[12..16]);
        let load_addr = BigEndian::read_u32(&uboot[16..20]);
        if data_size != uboot.len() as u32 - UBOOT_HEADER_SIZE {
            bail!(
                "U-Boot image data size mismatch: expected {:#010x}, got {:#010x}",
                uboot.len() as u32 - UBOOT_HEADER_SIZE,
                data_size
            );
        }
        // TODO (from sunxi-fel): Verify image data integrity using the checksum field `ih_dcrc`,
        // available from `BigEndian::read_u32(&uboot[24..28])`.
        //
        // However, this requires *CRC* routines that mimic their *U-Boot* counterparts, namely
        // `image_check_dcrc()` in `${U-BOOT}/common/image.c` and `crc_wd()` in
        // `${U-BOOT}/lib/crc32.c`.

        // If we get here, we're "good to go" (i.e. actually write the data)
        self.fel_write(load_addr, &uboot[UBOOT_HEADER_SIZE as usize..])
            .context("could not write the U-Boot image to the device")?;

        Ok((load_addr, data_size))
    }

    /// Writes *U-Boot* *SPL* into memory and executes it.
    pub fn write_and_execute_spl(&self, spl: &[u8]) -> Result<(), Error> {
        assert!(spl.len() >= 32, "SPL length must be bigger than 32 bytes");

        let mut spl_checksum = 2 * LittleEndian::read_u32(&spl[12..16]) - 0x5F0A_6C39;
        let spl_len = LittleEndian::read_u32(&spl[16..20]);

        if spl_len > spl.len() as u32 || (spl_len % 4) != 0 {
            return Err(FelError::SPLHeader {
                msg: "bad length in the provided SPL eGON header",
            }.into());
        }

        for i in spl.chunks(4) {
            spl_checksum = spl_checksum.wrapping_sub(LittleEndian::read_u32(i));
        }
        if spl_checksum != 0 {
            return Err(FelError::SPLHeader {
                msg: "the given SPL checksum does not match",
            }.into());
        }

        if self.soc_info.needs_l2en() {
            self.enable_l2_cache()
                .context("the SoC requires L2 cache but it couldn't be enabled")?;
        }

        let (_sp_irq, _sp) = self.get_stack_info()
            .context("could not retrieve stack information")?;

        let tt = match self.backup_and_disable_mmu()
            .context("could not backup and disable the MMU translation table")?
        {
            Some(tt) => Some(tt),
            None => {
                if let Some(mmu_tt_addr) = self.soc_info.get_mmu_tt_addr() {
                    if (mmu_tt_addr & 0x0000_3FFF) != 0 {
                        bail!("The MMU translation table address was not 16kB aligned");
                    }

                    // These settings are used by the *BROM* in A10/A13/A20 and we replicate them
                    // here when enabling the *MMU*. The `DACR` value `0x55555555` means that
                    // accesses are checked against the permission bits in the translation tables
                    // for all domains. The `TTBCR` value `0x00000000` means that the short
                    // descriptor translation table format is used, `TTBR0` is used for all the
                    // possible virtual addresses (`N=0`) and that the translation table must be
                    // aligned at a *16kB* boundary.
                    self.set_dacr(0x5555_5555).context("could not set DACR")?;
                    self.set_ttbcr(0x0000_0000).context("could not set TTBCR")?;
                    self.set_ttbr0(mmu_tt_addr).context("could not set TTBR0")?;

                    Some(FelHandle::default_mmu_translation_table())
                } else {
                    None
                }
            }
        };

        let mut cur_addr = self.soc_info.get_spl_addr();
        let mut spl_len_limit = SPL_LEN_LIMIT;
        let mut written = 0;
        let mut left = spl.len();
        for swap_buffers in self.soc_info.get_swap_buffers() {
            if swap_buffers.get_buf2() >= self.soc_info.get_spl_addr()
                && swap_buffers.get_buf2() < self.soc_info.get_spl_addr() + spl_len_limit
            {
                spl_len_limit = swap_buffers.get_buf2() - self.soc_info.get_spl_addr()
            }
            if left > 0 && cur_addr < swap_buffers.get_buf1() {
                let mut tmp = (swap_buffers.get_buf1() - cur_addr) as usize;
                if tmp > left {
                    tmp = left;
                }
                self.fel_write(cur_addr, &spl[written..tmp + written])
                    .context({
                        format!(
                            "could not write the file buffer to the current address ({:#010x})",
                            cur_addr
                        )
                    })?;
                cur_addr += tmp as u32;
                written += tmp;
                left -= tmp;
            }
            if left > 0 && cur_addr == swap_buffers.get_buf1() {
                let mut tmp = swap_buffers.get_size() as usize;
                if tmp > left {
                    tmp = left;
                }
                self.fel_write(swap_buffers.get_buf2(), &spl[written..tmp + written])
                    .context({
                        format!(
                            "could not write the file buffer to the second buffer ({:#010x})",
                            cur_addr
                        )
                    })?;
                cur_addr += tmp as u32;
                written += tmp;
                left -= tmp;
            }
        }

        // Clarify the SPL size limitations, and bail out if they are not met
        if self.soc_info.get_thunk_addr() < spl_len_limit {
            spl_len_limit = self.soc_info.get_thunk_addr();
        }

        if spl_len > spl_len_limit {
            bail!(
                "SPL too large, size limit is {} bytes, but it has {} bytes",
                spl_len_limit,
                spl_len
            );
        }

        // Write the remaining part of the SPL
        if left > 0 {
            self.fel_write(cur_addr, &spl[written..left + written])
                .context({
                    format!(
                        "could not write the file buffer to the current address ({:#010x})",
                        cur_addr
                    )
                })?;
        }

        let thunk_size = FEL_TO_SPL_THUNK.len() as u32 * 4 + 4
            + (self.soc_info.get_swap_buffers().len() as u32 + 1) * 12;
        if thunk_size > self.soc_info.get_thunk_size() {
            bail!(
                "bad thunk size, need {} bytes, but only {} bytes available",
                thunk_size,
                self.soc_info.get_thunk_size()
            );
        }

        // Generate thunk buffer.
        let mut thunk_buf = Vec::with_capacity((thunk_size / 4) as usize);
        thunk_buf.extend_from_slice(&FEL_TO_SPL_THUNK);
        thunk_buf.push(self.soc_info.get_spl_addr());
        for swap_buffers in self.soc_info.get_swap_buffers() {
            thunk_buf.push(swap_buffers.get_buf1());
            thunk_buf.push(swap_buffers.get_buf2());
            thunk_buf.push(swap_buffers.get_size());
        }
        if cfg!(not(target_endian = "little")) {
            for word in &mut thunk_buf {
                *word = word.to_le();
            }
        }
        thunk_buf.extend_from_slice(&[0_u32; 3]); // Empty buffer size

        // Execute SPL.
        self.fel_write(self.soc_info.get_thunk_addr(), u32_as_u8(&thunk_buf))
            .context("could not write thunk buffer to memory")?;
        self.fel_execute(self.soc_info.get_thunk_addr())
            .context("could not execute SPL code in thunk address")?;

        // TODO Try to find and fix the bug, which needs this workaround
        thread::sleep(Duration::from_millis(250));

        // Read back the result and check if everything was fine
        let mut signature = [0_u8; 8];
        self.fel_read(self.soc_info.get_spl_addr() + 4, &mut signature)
            .context("could not read SPL signature")?;
        if &signature != b"eGON.FEL" {
            bail!(format!(
                "unable to validate SPL load. Expected 'eGON.FEL' signature. Failure code: {}",
                String::from_utf8_lossy(&signature)
            ));
        }

        // Re-enable the MMU if it was enabled by BROM
        if let Some(tt) = tt {
            self.restore_and_enable_mmu(tt)
                .context("unable to restore and enable MMU")?;
        }
        Ok(())
    }
}

/// Utility function to determine the image type from a mkimage-compatible header at given
/// buffer.
fn get_image_type(image: &[u8]) -> Result<UbootImageType, Error> {
    debug_assert!(
        image.len() > UBOOT_HEADER_SIZE as usize,
        "insuficient image length"
    );
    if BigEndian::read_u32(&image[..4]) != UBOOT_IH_MAGIC {
        bail!("U-Boot signature mismatch");
    }
    if image[29] != UBOOT_IH_ARCH_ARM {
        bail!("invalid architechture in U-Boot header, only ARM is supported");
    }
    UbootImageType::from_byte(image[30])
}
