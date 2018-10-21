//! Allwinner FEL library.

#![cfg_attr(feature = "cargo-clippy", deny(clippy))]
#![forbid(anonymous_parameters)]
#![cfg_attr(feature = "cargo-clippy", warn(clippy_pedantic))]
#![deny(
    variant_size_differences,
    unused_results,
    unused_qualifications,
    unused_import_braces,
    unsafe_code,
    trivial_numeric_casts,
    trivial_casts,
    missing_docs,
    unused_extern_crates,
    missing_debug_implementations,
    missing_copy_implementations
)]
#![cfg_attr(
    feature = "cargo-clippy",
    allow(similar_names, cast_possible_truncation)
)]

#[macro_use]
extern crate failure;
extern crate byteorder;
extern crate libusb;

use std::io;
use std::ops::Deref;
use std::time::Duration;
use std::{fmt, u32};

use libusb::DeviceHandle;

use byteorder::{ByteOrder, LittleEndian};
use failure::{Error, Fail, ResultExt};

mod soc;
#[cfg(feature = "uboot")]
mod uboot;

/// FEL errors.
#[derive(Debug, Fail, PartialEq)]
pub enum FelError {
    /// USB response error.
    #[fail(
        display = "invalid response: expected '{}', found: {}",
        expected,
        found
    )]
    Response {
        /// Expected string.
        expected: &'static str,
        /// Found string.
        found: String,
    },
    /// Unsupported device ID.
    #[fail(display = "unsupported device ID: {:#010x}", id)]
    UnsupportedDevId {
        /// Unsupported device ID.
        id: u32,
    },
    /// SPL header error.
    #[fail(display = "SPL header error: {}", msg)]
    SPLHeader {
        /// SPL header error message.
        msg: &'static str,
    },
}

/// Maximum size of *SPL*, at the same time this is the start offset of the main *U-Boot* image
/// within `u-boot-sunxi-with-spl.bin`.
#[cfg(feature = "uboot")]
pub const SPL_LEN_LIMIT: u32 = 0x8000;

/// USB timeout (in seconds).
const USB_TIMEOUT: u64 = 10;
/// `AW_USB_MAX_BULK_SEND` and the timeout constant `USB_TIMEOUT` are related. Both need to be
/// selected in a way that transferring the maximum chunk size with (*SoC*-specific) slow transfer
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

/// *DRAM* base address.
const DRAM_BASE: u32 = 0x4000_0000;
/// *DRAM* size, in bytes.
const DRAM_SIZE: u32 = 0x8000_0000;

/// Converts a reference to a `u32` slice to a reference to a `u8` slice.
///
/// The new slice will have a 4 times bigger length.
#[inline(always)]
#[allow(unsafe_code)]
fn u32_as_u8(src: &[u32]) -> &[u8] {
    // Safe because both slices have the same size in bytes.
    unsafe { std::slice::from_raw_parts(src.as_ptr() as *mut u8, src.len() * 4) }
}

/// Converts a mutable reference to a `u32` slice to a mutable reference to a `u8` slice.
///
/// The new slice will have a 4 times bigger length.
#[inline(always)]
#[allow(unsafe_code)]
fn u32_as_u8_mut(src: &mut [u32]) -> &mut [u8] {
    // Safe because both slices have the same size in bytes.
    unsafe { std::slice::from_raw_parts_mut(src.as_mut_ptr() as *mut u8, src.len() * 4) }
}

/// FEL device handle.
pub struct FelHandle<'h> {
    usb_handle: UsbHandle<'h>,
    soc_version: soc::Version,
    soc_info: soc::Info,
}

impl<'h> fmt::Debug for FelHandle<'h> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("FelHandle")
            .field("usb_handle", &self.usb_handle)
            .field("soc_version", &self.soc_version)
            .field("soc_info", &self.soc_info)
            .finish()
    }
}

impl<'h> FelHandle<'h> {
    /// Gets the SoC information from the FEL device.
    ///
    /// Note: This is a no-op. The *SoC* information is acquired during initialization.
    pub fn get_soc_info(&self) -> &soc::Info {
        &self.soc_info
    }

    /// Gets the *SoC* version information from the *FEL* device.
    ///
    /// Note: This is a no-op. The SoC information is acquired during initialization.
    pub fn get_version_info(&self) -> &soc::Version {
        &self.soc_version
    }

    /// Enables the L2 cache.
    fn enable_l2_cache(&self) -> Result<(), Error> {
        let arm_code: [u32; 4] = [
            // mrc        15, 0, r2, cr1, cr0, {1}
            0x_ee_11_2f_30_u32.to_le(),
            // orr        r2, r2, #2
            0x_e3_82_20_02_u32.to_le(),
            // mcr        15, 0, r2, cr1, cr0, {1}
            0x_ee_01_2f_30_u32.to_le(),
            // Return back to FEL
            // bx         lr
            0x_e1_2f_ff_1e_u32.to_le(),
        ];
        self.fel_write(self.soc_info.get_scratch_addr(), u32_as_u8(&arm_code))
            .context("could not write L2 cache enabling ARM code")?;
        self.fel_execute(self.soc_info.get_scratch_addr())
            .context("could not execute L2 cache enabling ARM code")?;
        Ok(())
    }

    /// Gets stack information.
    ///
    /// The result will be a tuple. The first element will be the `SP_irq`, while the second element
    /// will be the nuser mode `SP`.
    fn get_stack_info(&self) -> Result<(u32, u32), Error> {
        let arm_code: [u32; 9] = [
            // mrs        r0, CPSR
            0x_e1_0f_00_00_u32.to_le(),
            // bic        r1, r0, #31
            0x_e3_c0_10_1f_u32.to_le(),
            // orr        r1, r1, #18
            0x_e3_81_10_12_u32.to_le(),
            // msr        CPSR_c, r1
            0x_e1_21_f0_01_u32.to_le(),
            // mov        r1, sp
            0x_e1_a0_10_0d_u32.to_le(),
            // msr        CPSR_c, r0
            0x_e1_21_f0_00_u32.to_le(),
            // str        r1, [pc, #4]
            0x_e5_8f_10_04_u32.to_le(),
            // str        sp, [pc, #4]
            0x_e5_8f_d0_04_u32.to_le(),
            // Return back to FEL
            // bx         lr
            0x_e1_2f_ff_1e_u32.to_le(),
        ];
        self.fel_write(self.soc_info.get_scratch_addr(), u32_as_u8(&arm_code))
            .context("could not write ARM code to read stack information")?;
        self.fel_execute(self.soc_info.get_scratch_addr())
            .context("could not execute ARM code to get stack information")?;
        let mut result = [0_u8; 2 * 4];
        self.fel_read(self.soc_info.get_scratch_addr() + 9 * 4, &mut result)
            .context("could not read generated stack information")?;
        Ok((
            LittleEndian::read_u32(&result[..4]),
            LittleEndian::read_u32(&result[4..]),
        ))
    }

    /// Generates a default *MMU* translation table.
    fn default_mmu_translation_table() -> [u32; 4 * 1024] {
        let mut tt = [0_u32; 4 * 1024];
        for (i, word) in tt.iter_mut().enumerate() {
            *word = 0x0000_0DE2 | ((i as u32) << 20);
            if i == 0x000 || i == 0xFFF {
                *word |= 0x1000;
            }
        }
        tt
    }

    /// Backup *MMU* translation table and disable it.
    fn backup_and_disable_mmu(&self) -> Result<Option<[u32; 4 * 1024]>, Error> {
        // Below are some checks for the register values, which are known to be initialized in this
        // particular way by the existing BROM implementations. We don't strictly need them to
        // exactly match, but still have these safety guards in place in order to detect and review
        // any potential configuration changes in future SoC variants (if one of these checks fails,
        // then it is not a serious problem but more likely just an indication that one of these
        // check needs to be relaxed).

        // Basically, ignore M/Z/I/V/UNK bits and expect no TEX remap.
        // Check `SCTLR` register.
        let sctlr = self.get_sctlr().context("unable to read SCTLR register")?;
        if (sctlr & !((0x7 << 11) | (1 << 6) | 1)) != 0x00C5_0038 {
            bail!("unexpected SCTLR register ({:#010x})", sctlr);
        }
        if (sctlr & 0x0000_0001) == 0 {
            return Ok(None);
        }

        // Check `DACR` register.
        let dacr = self.get_dacr().context("unable to read DACR register")?;
        if dacr != 0x5555_5555 {
            bail!("unexpected DACR register ({:#010x})", dacr);
        }

        // Check `TTBRC` register.
        let ttbcr = self.get_ttbcr().context("unable to read TTBCR register")?;
        if ttbcr != 0x0000_0000 {
            bail!("unexpected TTBCR register ({:#010x})", ttbcr);
        }

        // Check `TTBR0` register
        let ttbr0 = self.get_ttbr0().context("unable to read TTBR0 register")?;
        if (ttbr0 & 0x0000_3FFF) != 0 {
            bail!("unexpected TTBR0 register ({:#010x})", ttbr0);
        }

        // Read MMU translation table.
        let mut tt = [0_u32; 4 * 1024];
        self.fel_read(ttbr0, u32_as_u8_mut(&mut tt)).context({
            format!(
                "could not read the MMU translation table from {:#010x}",
                ttbr0
            )
        })?;
        for (i, le_word) in tt.iter_mut().enumerate() {
            let word = u32::from_le(*le_word);

            // Sanity checks:
            if ((word >> 1) & 1) != 1 || ((word >> 18) & 1) != 0 {
                bail!("found a word in the translation table that was not a section descriptor");
            }
            if (word >> 20) != i as u32 {
                bail!("found a word in the translation table that was not a direct mapping");
            }
            *le_word = word;
        }

        // Disable I-cache, MMU and branch prediction
        let arm_code: [u32; 6] = [
            // mrc        15, 0, r0, cr1, cr0, {0}
            0x_ee_11_0f_10_u32.to_le(),
            // bic        r0, r0, #1
            0x_e3_c0_00_01_u32.to_le(),
            // bic        r0, r0, #4096
            0x_e3_c0_0a_01_u32.to_le(),
            // bic        r0, r0, #2048
            0x_e3_c0_0b_02_u32.to_le(),
            // mcr        15, 0, r0, cr1, cr0, {0}
            0x_ee_01_0f_10_u32.to_le(),
            // Return back to FEL
            // bx         lr
            0x_e1_2f_ff_1e_u32.to_le(),
        ];

        self.fel_write(self.soc_info.get_scratch_addr(), u32_as_u8(&arm_code))
            .context({
                "could not write ARM code to memory for disabling I-cache, MMU and branch \
                 prediction"
            })?;
        self.fel_execute(self.soc_info.get_scratch_addr())
            .context({
                "could not execute ARM code to memory to disable I-cache, MMU and branch prediction"
            })?;
        Ok(Some(tt))
    }

    // Restore and enable MMU.
    fn restore_and_enable_mmu(&self, mut tt: [u32; 4 * 1024]) -> Result<(), Error> {
        let ttbr0 = self
            .get_ttbr0()
            .context("unable to read `TBBR0` register")?;

        // Setting write-combine mapping for *DRAM*.
        let start = (DRAM_BASE >> 20) as usize;
        let end = ((DRAM_BASE + DRAM_SIZE) >> 20) as usize;
        for word in tt[start..end].iter_mut() {
            // Clear `TEXCB` bits
            *word &= !((7 << 12) | (1 << 3) | (1 << 2));
            // Set `TEXCB` to `00100` (Normal uncached mapping)
            *word |= 1 << 12;
        }

        // Setting cached mappint for *BROM*.
        // Clear `TEXCB` bits first
        tt[0xFFF] &= !((7 << 12) | (1 << 3) | (1 << 2));
        // Set `TEXCB` to `00111` (Normal write-back cached mapping)
        tt[0xFFF] |= (1 << 12) | // TEX
                     (1 << 3) | // C
                     (1 << 2); // B

        if cfg!(not(target_endian = "little")) {
            for word in tt.iter_mut() {
                *word = word.to_le();
            }
        }

        self.fel_write(ttbr0, u32_as_u8(&tt))
            .context("could not write MMU translation table to memory")?;

        // Enabling I-cache, MMU and branch prediction...
        let arm_code: [u32; 12] = [
            // Invalidate I-cache, TLB and BTB
            // mov        r0, #0
            0x_e3_a0_00_00_u32.to_le(),
            // mcr        15, 0, r0, cr8, cr7, {0}
            0x_ee_08_0f_17_u32.to_le(),
            // mcr        15, 0, r0, cr7, cr5, {0}
            0x_ee_07_0f_15_u32.to_le(),
            // mcr        15, 0, r0, cr7, cr5, {6}
            0x_ee_07_0f_d5_u32.to_le(),
            // dsb        sy
            0x_f5_7f_f0_4f_u32.to_le(),
            // isb        sy
            0x_f5_7f_f0_6f_u32.to_le(),
            // Enable I-cache, MMU and branch prediction
            // mrc        15, 0, r0, cr1, cr0, {0}
            0x_ee_11_0f_10_u32.to_le(),
            // orr        r0, r0, #1
            0x_e3_80_00_01_u32.to_le(),
            // orr        r0, r0, #4096
            0x_e3_80_0a_01_u32.to_le(),
            // orr        r0, r0, #2048
            0x_e3_80_0b_02_u32.to_le(),
            // mcr        15, 0, r0, cr1, cr0, {0}
            0x_ee_01_0f_10_u32.to_le(),
            // Return back to FEL
            // bx         lr
            0x_e1_2f_ff_1e_u32.to_le(),
        ];

        self.fel_write(self.soc_info.get_scratch_addr(), u32_as_u8(&arm_code))
            .context("could not write MMU enablement code to device memory")?;
        self.fel_execute(self.soc_info.get_scratch_addr())
            .context("could not execute the MMU enablement code")?;
        Ok(())
    }

    /// Gets the `TTBR0` register.
    fn get_ttbr0(&self) -> Result<u32, Error> {
        self.read_arm_cp_reg(15, 0, 2, 0, 0)
    }

    /// Sets the `TTBR0` register to the given value.
    fn set_ttbr0(&self, val: u32) -> Result<(), Error> {
        self.write_arm_cp_reg(15, 0, 2, 0, 0, val)
    }

    /// Gets the `TTBCR` register.
    fn get_ttbcr(&self) -> Result<u32, Error> {
        self.read_arm_cp_reg(15, 0, 2, 0, 2)
    }

    /// Sets the `TTBCR` register to the given value.
    fn set_ttbcr(&self, val: u32) -> Result<(), Error> {
        self.write_arm_cp_reg(15, 0, 2, 0, 2, val)
    }

    /// Gets the `DACR` register.
    fn get_dacr(&self) -> Result<u32, Error> {
        self.read_arm_cp_reg(15, 0, 3, 0, 0)
    }

    /// Sets the `DACR` register to the given value.
    fn set_dacr(&self, val: u32) -> Result<(), Error> {
        self.write_arm_cp_reg(15, 0, 3, 0, 0, val)
    }

    /// Gets the `SCTLR` register.
    fn get_sctlr(&self) -> Result<u32, Error> {
        self.read_arm_cp_reg(15, 0, 1, 0, 0)
    }

    /// Sets the `SCTLR` register to the given value.
    #[allow(dead_code)]
    fn set_sctlr(&self, val: u32) -> Result<(), Error> {
        self.write_arm_cp_reg(15, 0, 1, 0, 0, val)
    }

    /// Reads the given ARM register.
    fn read_arm_cp_reg(
        &self,
        coproc: u32,
        opc1: u32,
        cr_n: u32,
        cr_m: u32,
        opc2: u32,
    ) -> Result<u32, Error> {
        let opcode = 0xEE00_0000
            | (1 << 20)
            | (1 << 4)
            | ((opc1 & 7) << 21)
            | ((cr_n & 15) << 16)
            | ((coproc & 15) << 8)
            | ((opc2 & 7) << 5)
            | (cr_m & 15);
        let arm_code: [u32; 3] = [
            // mrc  coproc, opc1, r0, cr_n, cr_m, opc2
            opcode.to_le(),
            // str  r0, [pc]
            0x_e5_8f_00_00_u32.to_le(),
            // bx   lr
            0x_e1_2f_ff_1e_u32.to_le(),
        ];
        self.fel_write(self.soc_info.get_scratch_addr(), u32_as_u8(&arm_code))
            .context("could not write ARM code to read register")?;
        self.fel_execute(self.soc_info.get_scratch_addr())
            .context("could not execute ARM code to read register")?;
        let mut reg_value = [0_u8; 4];
        self.fel_read(self.soc_info.get_scratch_addr() + 3 * 4, &mut reg_value)
            .context("could not read the register information from memory")?;
        Ok(LittleEndian::read_u32(&reg_value))
    }

    /// Writes the given value to the given ARM register.
    fn write_arm_cp_reg(
        &self,
        coproc: u32,
        opc1: u32,
        cr_n: u32,
        cr_m: u32,
        opc2: u32,
        val: u32,
    ) -> Result<(), Error> {
        let opcode = 0xEE00_0000
            | (1 << 4)
            | ((opc1 & 7) << 21)
            | ((cr_n & 15) << 16)
            | ((coproc & 15) << 8)
            | ((opc2 & 7) << 5)
            | (cr_m & 15);
        let arm_code: [u32; 6] = [
            // ldr  r0, [pc, #12]
            0x_e5_9f_00_0c_u32.to_le(),
            // mcr  coproc, opc1, r0, cr_n, cr_m, opc2
            opcode.to_le(),
            // dsb  sy
            0x_f5_7f_f0_4f_u32.to_le(),
            // isb  sy
            0x_f5_7f_f0_6f_u32.to_le(),
            // bx   lr
            0x_e1_2f_ff_1e_u32.to_le(),
            val.to_le(),
        ];
        self.fel_write(self.soc_info.get_scratch_addr(), u32_as_u8(&arm_code))
            .context("could not write ARM code to write to register")?;
        self.fel_execute(self.soc_info.get_scratch_addr())
            .context("could not execute ARM code to write to register")?;
        Ok(())
    }

    /// Reads the SID from the SoC if it has one.
    pub fn read_sid(&self) -> Result<Option<[u32; 4]>, Error> {
        if let Some(sid_addr) = self.soc_info.get_sid_addr() {
            let mut sid = [0_u32; 4];
            self.read_words(sid_addr, &mut sid)
                .context("unable to read SID registers")?;
            Ok(Some(sid))
        } else {
            Ok(None)
        }
    }

    /// Read words chunk from the FEL device memory.
    pub fn read_words(&self, offset: u32, words: &mut [u32]) -> Result<(), Error> {
        assert!(
            u32::max_value() - (words.len() * 4) as u32 > offset,
            "cannot read above {:#010x} - offset: {:#010x}, buffer length: {:#010x} words, words \
             above offset: {:#010x}",
            u32::max_value(),
            offset,
            words.len(),
            (u32::max_value() - offset) / 4
        );
        for (i, chunk) in words.chunks_mut(LCODE_MAX_RW_WORDS).enumerate() {
            self.read_words_chunk(offset + (i * LCODE_ARM_RW_WORDS) as u32, chunk)
                .context("could not read word chunk from memory")?;
        }
        Ok(())
    }

    /// Write words chunk to the FEL device memory.
    pub fn write_words(&self, offset: u32, words: &[u32]) -> Result<(), Error> {
        assert!(
            u32::max_value() - (words.len() * 4) as u32 > offset,
            "cannot write above {:#010x} - offset: {:#010x}, buffer length: {:#010x} words, words \
             above offset: {:#010x}",
            u32::max_value(),
            offset,
            words.len(),
            (u32::max_value() - offset) / 4
        );
        for (i, chunk) in words.chunks(LCODE_MAX_RW_WORDS).enumerate() {
            self.write_words_chunk(offset + (i * LCODE_ARM_RW_WORDS) as u32, chunk)
                .context("could not write word chunk to memory")?;
        }
        Ok(())
    }

    /// Read words chunk from the FEL device memory.
    fn read_words_chunk(&self, offset: u32, words: &mut [u32]) -> Result<(), Error> {
        debug_assert!(
            words.len() <= LCODE_MAX_RW_WORDS,
            "read_words requests cannot exceed {} words",
            LCODE_MAX_RW_WORDS
        );
        let arm_code: [u32; LCODE_ARM_RW_WORDS] = [
            // `ldr   r0, [pc, #32] ; ldr r0,[read_addr]`
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
            (words.len() as u32).to_le(),
        ];
        // read_data (buffer) follows, i.e. values go here.

        // scratch buffer setup: transfers ARM code, including addr and count
        self.fel_write(self.soc_info.get_scratch_addr(), u32_as_u8(&arm_code))
            .context("unable to write ARM code to scratch address")?;
        // execute code, read back the result
        self.fel_execute(self.soc_info.get_scratch_addr())
            .context("unable to execute ARM code")?;
        self.fel_read(
            self.soc_info.get_scratch_addr() + LCODE_ARM_RW_SIZE as u32,
            u32_as_u8_mut(words),
        ).context("unable to read generated buffer")?;

        if cfg!(not(target_endian = "little")) {
            for word in words.iter_mut() {
                *word = word.to_le();
            }
        }

        Ok(())
    }

    /// Write words chunk to the FEL device memory.
    fn write_words_chunk(&self, offset: u32, words: &[u32]) -> Result<(), Error> {
        debug_assert!(
            words.len() <= LCODE_MAX_RW_WORDS,
            "write_words requests cannot exceed {} words",
            LCODE_MAX_RW_WORDS
        );

        let mut arm_code = Vec::with_capacity(LCODE_ARM_RW_WORDS + words.len());
        arm_code.extend_from_slice(&[
            // `ldr   r0, [pc, #32] ; ldr r0,[write_addr]`
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
            (words.len() as u32).to_le(),
        ]);
        // write_data (buffer) follows, i.e. values go here.
        // copy values from source buffer
        for word in words.iter() {
            arm_code.push(word.to_le());
        }

        // scratch buffer setup: transfers ARM code and data
        self.fel_write(self.soc_info.get_scratch_addr(), u32_as_u8(&arm_code))
            .context("unable to write ARM code to scratch address")?;
        // execute code
        self.fel_execute(self.soc_info.get_scratch_addr())
            .context("unable to execute ARM code")?;

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
    pub fn rmr_request(&self, entry_point: u32, aarch64: bool) -> Result<(), Error> {
        if let Some(rvbar_reg) = self.soc_info.get_rvbar_reg() {
            let rmr_mode: u32 = (1 << 1) | (if aarch64 { 1 } else { 0 });

            let arm_code: [u32; LCODE_ARM_RMR_WORDS] = [
                // ldr    r0, [rvbar_reg]
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
                rmr_mode.to_le(),
            ];

            self.fel_write(self.soc_info.get_scratch_addr(), u32_as_u8(&arm_code))
                .context("unable to write ARM code to scratch address")?;
            self.fel_execute(self.soc_info.get_scratch_addr())
                .context("unable to execute ARM code")?;
            Ok(())
        } else {
            bail!(
                "can't issue RMR request, the SoC ({}) does not support RVBAR or it's unknown",
                self.soc_info.get_name()
            );
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

impl<'h> fmt::Debug for UsbHandle<'h> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("UsbHandle")
            .field("endpoint_in", &self.endpoint_in)
            .field("endpoint_out", &self.endpoint_out)
            .field("iface_detached", &self.iface_detached)
            .finish()
    }
}

impl<'h> UsbHandle<'h> {
    /// Creates a USB handle from the given device.
    fn from_device(device: &libusb::Device<'h>) -> Result<UsbHandle<'h>, Error> {
        let mut handle = UsbHandle {
            device_handle: device.open().context("unable to open device")?,
            endpoint_in: 0,
            endpoint_out: 0,
            iface_detached: false,
        };
        handle.claim(&device)?;
        Ok(handle)
    }

    /// Claim the given device.
    fn claim(&mut self, device: &libusb::Device<'h>) -> Result<(), Error> {
        if let Err(e) = self.device_handle.claim_interface(0) {
            if cfg!(target_os = "linux") {
                self.device_handle
                    .detach_kernel_driver(0)
                    .context("unable to detach kernel driver")?;
                self.iface_detached = true;
                self.device_handle
                    .claim_interface(0)
                    .context("unable to claim device interface")?;
            } else {
                bail!(e.context("unable to claim device interface"));
            }
        }
        self.get_endpoints(&device)
            .context("unable to get device endpoints")?;
        Ok(())
    }

    /// Updates the endpoints of the USB handle.
    fn get_endpoints(&mut self, device: &libusb::Device<'h>) -> Result<(), Error> {
        use libusb::{Direction, TransferType};

        let config_descriptor = device
            .active_config_descriptor()
            .context("unable to get active config descriptor")?;
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
    pub fn fel_fill(&self, offset: u32, num_bytes: u32, byte: u8) -> Result<(), Error> {
        assert!(
            u32::max_value() - num_bytes > offset,
            "cannot write above {:#010x} - offset: {:#010x}, num_bytes: {:#010x}, bytes above \
             offset: {:#010x}",
            u32::max_value(),
            offset,
            num_bytes,
            u32::max_value() - offset
        );
        let buf = vec![byte; num_bytes as usize];
        self.fel_write(offset, &buf)
            .context("unable to write filling buffer to device memory")?;
        Ok(())
    }

    /// Reads from the FEL device memory at the given offset.
    ///
    /// It will fill all the given buffer, but it **will panic** if the buffer overflows total
    /// memory address space.
    pub fn fel_read(&self, offset: u32, buf: &mut [u8]) -> Result<(), Error> {
        assert!(
            u32::max_value() - buf.len() as u32 > offset,
            "cannot read above {:#010x} - offset: {:#010x}, buffer length: {:#010x}, bytes above \
             offset: {:#010x}",
            u32::max_value(),
            offset,
            buf.len(),
            u32::max_value() - offset
        );
        self.send_fel_request(AW_FEL_1_READ, offset, buf.len() as u32)
            .context("unable to send AW_FEL_1_READ FEL request")?;
        self.usb_read(buf).context("unable read data from USB")?;
        self.read_fel_status()
            .context("unable to read FEL status")?;
        Ok(())
    }

    /// Writes to the FEL device memory at the given offset.
    ///
    /// It **will panic** if the buffer overflows total memory address space.
    pub fn fel_write(&self, offset: u32, buf: &[u8]) -> Result<(), Error> {
        assert!(
            u32::max_value() - buf.len() as u32 > offset,
            "cannot write above {:#010x} - offset: {:#010x}, buffer length: {:#010x}, bytes above \
             offset: {:#010x}",
            u32::max_value(),
            offset,
            buf.len(),
            u32::max_value() - offset
        );
        self.send_fel_request(AW_FEL_1_WRITE, offset, buf.len() as u32)
            .context("unable to send AW_FEL_1_WRITER FEL request")?;
        self.usb_write(buf).context("unable write data to USB")?;
        self.read_fel_status()
            .context("unable to read FEL status")?;
        Ok(())
    }

    /// Makes the FEL device execute the code at the given address.
    pub fn fel_execute(&self, offset: u32) -> Result<(), Error> {
        self.send_fel_request(AW_FEL_1_EXEC, offset, 0)
            .context("unable to send AW_FEL_1_EXEC FEL request")?;
        self.read_fel_status()
            .context("unable to read FEL status")?;
        Ok(())
    }

    /// Gets the SoC version information.
    fn get_fel_version(&self) -> Result<soc::Version, Error> {
        self.send_fel_request(AW_FEL_VERSION, 0, 0)
            .context("unable to send AW_FEL_VERSION FEL request")?;
        let mut buf = [0_u8; 32];
        self.usb_read(&mut buf)
            .context("unable to read version from USB")?;
        self.read_fel_status()
            .context("unable to read FEL status")?;

        Ok(soc::Version::from_bytes(buf))
    }

    /// Reads the FEL status.
    fn read_fel_status(&self) -> Result<(), Error> {
        let mut buf = [0_u8; 8];
        self.usb_read(&mut buf).context("unable to read from USB")?;
        Ok(())
    }

    /// Sends a FEL request.
    fn send_fel_request(&self, req_type: u32, addr: u32, len: u32) -> Result<(), Error> {
        use byteorder::{ByteOrder, LittleEndian};

        let mut request = [0_u8; 16];
        LittleEndian::write_u32(&mut request[..4], req_type);
        LittleEndian::write_u32(&mut request[4..8], addr);
        LittleEndian::write_u32(&mut request[8..12], len);
        // 4 byte padding.

        self.usb_write(&request).context("unable to write to USB")?;
        Ok(())
    }

    /// Reads the given data from the USB.
    fn usb_read(&self, data: &mut [u8]) -> Result<(), Error> {
        self.send_usb_request(AW_USB_READ, data.len() as u32)
            .context("unable to send AW_USB_READ USB request")?;
        UsbHandle::usb_bulk_recv(&self.device_handle, self.endpoint_in, data).context({
            "unable to receive data in bulk from USB after AW_USB_READ USB \
             request"
        })?;
        self.read_usb_response()
            .context("unable to read response from USB")?;
        Ok(())
    }

    /// Writes the given data to the USB.
    fn usb_write(&self, data: &[u8]) -> Result<(), Error> {
        self.send_usb_request(AW_USB_WRITE, data.len() as u32)
            .context("unable to send AW_USB_WRITE USB request")?;
        UsbHandle::usb_bulk_send(&self.device_handle, self.endpoint_out, data)
            .context("unable to send data in bulk to USB after AW_USB_WRITE USB request")?;
        self.read_usb_response()
            .context("unable to read response from USB")?;
        Ok(())
    }

    /// Sends the request type USB request.
    fn send_usb_request(&self, req_type: u16, len: u32) -> Result<(), Error> {
        use byteorder::{ByteOrder, LittleEndian};

        let mut request = [0_u8; 32];
        request[..4].clone_from_slice(b"AWUC");
        LittleEndian::write_u32(&mut request[8..12], len);
        LittleEndian::write_u32(&mut request[12..16], 0x0C00_0000); // Unknown data
        LittleEndian::write_u16(&mut request[16..18], req_type);
        LittleEndian::write_u32(&mut request[18..22], len); // Length is repeated
                                                            // Bytes from 22 to 32 are padding

        UsbHandle::usb_bulk_send(&self.device_handle, self.endpoint_out, &request)
            .context("unable to send data in bulk to USB")?;
        Ok(())
    }

    /// Reads and discards the USB response. It will check if the response is correct.
    fn read_usb_response(&self) -> Result<(), Error> {
        let mut buf = [0_u8; 13];
        UsbHandle::usb_bulk_recv(&self.device_handle, self.endpoint_in, &mut buf)
            .context("unable to receive data in bulk from USB")?;
        if &buf[..4] == b"AWUS" {
            Ok(())
        } else {
            Err(FelError::Response {
                expected: "AWUS[...]",
                found: format!("{}[...]", String::from_utf8_lossy(&buf[..4])),
            }.into())
        }
    }

    /// Sends data to the USB in bulk.
    ///
    /// It will divide the data in chunks.
    fn usb_bulk_send(device_handle: &DeviceHandle, endpoint: u8, data: &[u8]) -> Result<(), Error> {
        let mut sent = 0;
        while sent < data.len() {
            let slice = if data.len() - sent < AW_USB_MAX_BULK_SEND {
                &data[sent..]
            } else {
                &data[sent..sent + AW_USB_MAX_BULK_SEND]
            };
            sent += device_handle
                .write_bulk(endpoint, slice, Duration::from_secs(USB_TIMEOUT))
                .context({
                    format!(
                        "unable to send data chunk to USB, sent {} bytes: {:?}",
                        sent,
                        &data[..sent]
                    )
                })?;
        }

        Ok(())
    }

    /// Receives data from the USB in bulk.
    ///
    /// It will divide the data in chunks.
    fn usb_bulk_recv(
        device_handle: &DeviceHandle,
        endpoint: u8,
        data: &mut [u8],
    ) -> Result<(), Error> {
        let mut received = 0;
        while received < data.len() {
            let slice = if data.len() - received < AW_USB_MAX_BULK_SEND {
                &mut data[received..]
            } else {
                &mut data[received..received + AW_USB_MAX_BULK_SEND]
            };
            received += device_handle
                .read_bulk(endpoint, slice, Duration::from_secs(USB_TIMEOUT))
                .context({
                    format!(
                        "unable to read data chunk from USB, received {} bytes",
                        received
                    )
                })?;
        }
        Ok(())
    }
}

impl<'h> Drop for UsbHandle<'h> {
    fn drop(&mut self) {
        use std::error::Error;
        use std::io::Write;
        if let Err(e) = self.device_handle.release_interface(0) {
            io::stderr()
                .write_all(
                    format!(
                        "error releasing device handle interface: {:?} ({})",
                        e,
                        e.description()
                    ).as_bytes(),
                ).unwrap();
        }
        if cfg!(target_os = "linux") && self.iface_detached {
            if let Err(e) = self.device_handle.attach_kernel_driver(0) {
                io::stderr()
                    .write_all(
                        format!(
                            "error attaching kernel driver: {:?} ({})",
                            e,
                            e.description()
                        ).as_bytes(),
                    ).unwrap();
            }
        }
    }
}

/// Allwinner FEL devices context.
pub struct Fel {
    context: libusb::Context,
}

impl fmt::Debug for Fel {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("Fel").finish()
    }
}

impl Fel {
    /// Creates a new Fel object.
    pub fn new() -> Result<Self, Error> {
        Ok(Self {
            context: libusb::Context::new().context("unable to create libUSB context")?,
        })
    }

    /// Get the device from the given bus and address, if it exists and is a FEL device.
    pub fn get_device(&self, bus: u8, address: u8) -> Result<Option<FelHandle>, Error> {
        for device in self
            .context
            .devices()
            .context("unable to list USB devices")?
            .iter()
        {
            if device.bus_number() == bus && device.address() == address {
                let device_descriptor = device
                    .device_descriptor()
                    .context("unable to get USB device descriptor")?;
                if device_descriptor.vendor_id() == AW_VENDOR_ID
                    && device_descriptor.product_id() == AW_PRODUCT_ID
                {
                    let usb_handle = UsbHandle::from_device(&device)?;
                    let soc_version = usb_handle.get_fel_version()?;
                    return if let Some(soc_info) = soc::Info::from_version(&soc_version) {
                        Ok(Some(FelHandle {
                            usb_handle,
                            soc_version,
                            soc_info,
                        }))
                    } else {
                        Err(FelError::UnsupportedDevId {
                            id: soc_version.get_id(),
                        }.into())
                    };
                } else {
                    return Ok(None);
                }
            }
        }
        Ok(None)
    }

    /// Creates a list of Allwinner devices in Fel mode.
    pub fn list_devices(&self) -> Result<Vec<FelHandle>, Error> {
        let mut result = Vec::new();
        for device in self
            .context
            .devices()
            .context("unable to list USB devices")?
            .iter()
        {
            let device_descriptor = device
                .device_descriptor()
                .context("unable to get USB device descriptor")?;
            if device_descriptor.vendor_id() == AW_VENDOR_ID
                && device_descriptor.product_id() == AW_PRODUCT_ID
            {
                let usb_handle = UsbHandle::from_device(&device)?;
                let soc_version = usb_handle.get_fel_version()?;
                if let Some(soc_info) = soc::Info::from_version(&soc_version) {
                    result.push(FelHandle {
                        usb_handle,
                        soc_version,
                        soc_info,
                    })
                } else {
                    return Err(FelError::UnsupportedDevId {
                        id: soc_version.get_id(),
                    }.into());
                }
            }
        }
        Ok(result)
    }
}
