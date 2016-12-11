//! *SoC* module.

use std::fmt;

/// Allwinner *A10*, *A13* and *A20* *SRAM* swap buffers.
///
/// The *FEL* code from *BROM* in *A10/A13/A20* sets up two stacks for itself. One at `0x2000` (and
/// growing down) for the *IRQ* handler. And another one at `0x7000` (and also growing down) for
/// the regular code. In order to use the whole *32 KiB* in the *A1/A2* sections of *SRAM*, we need
/// to temporarily move these stacks elsewhere. And the addresses `0x7D00`–`0x7FFF` contain
/// something important too (overwriting them kills *FEL*). On *A10/A13/A20* we can use the *SRAM*
/// sections *A3/A4* (`0x8000`–`0xBFFF`) for this purpose.
const A10_A13_A20_SRAM_SWAP_BUFFERS: [SRAMSwapBuffers; 3] = [// `0x1C00`–`0x1FFF` (IRQ stack)
                                                             SRAMSwapBuffers {
                                                                 buf1: 0x1C00,
                                                                 buf2: 0xA400,
                                                                 size: 0x0400,
                                                             },
                                                             // `0x5C00`–`0x6FFF` (Stack)
                                                             SRAMSwapBuffers {
                                                                 buf1: 0x5C00,
                                                                 buf2: 0xA800,
                                                                 size: 0x1400,
                                                             },
                                                             // `0x7C00`–`0x7FFF`
                                                             // (Something important)
                                                             SRAMSwapBuffers {
                                                                 buf1: 0x7C00,
                                                                 buf2: 0xBC00,
                                                                 size: 0x0400,
                                                             }];

/// Allwinner *A31* *SRAM* swap buffers.
///
/// *A31* is very similar to *A10/A13/A20*, except that it has no *SRAM* at `0x8000`. So we use the
/// *SRAM* section *B* at `0x20000`–`0x2FFFF` instead. In the *FEL* mode, the *MMU* translation
/// table is allocated by the *BROM* at `0x20000`. But we can also safely use it as the backup
/// storage because the *MMU* is temporarily disabled during the time of the *SPL* execution.
const A31_SRAM_SWAP_BUFFERS: [SRAMSwapBuffers; 2] = [SRAMSwapBuffers {
                                                         buf1: 0x1800,
                                                         buf2: 0x20000,
                                                         size: 0x800,
                                                     },
                                                     SRAMSwapBuffers {
                                                         buf1: 0x5C00,
                                                         buf2: 0x20800,
                                                         size: 0x8000 - 0x5C00,
                                                     }];

/// Allwinner *A64* *SRAM* swap buffers.
///
/// *A64* has *32KiB* of *SRAM* *A* at `0x10000` and a large *SRAM* *C* at `0x18000`. *SRAM* *A*
/// and *SRAM* *C* reside in the address space back-to-back without any gaps, thus representing a
/// singe large contiguous area. Everything is the same as on *A10/A13/A20*, but just shifted by
/// `0x10000`.
const A64_SRAM_SWAP_BUFFERS: [SRAMSwapBuffers; 3] = [// `0x11C00`–`0x11FFF` (IRQ stack)
                                                     SRAMSwapBuffers {
                                                         buf1: 0x11C00,
                                                         buf2: 0x1A400,
                                                         size: 0x0400,
                                                     },
                                                     // `0x15C00`–`0x16FFF` (Stack)
                                                     SRAMSwapBuffers {
                                                         buf1: 0x15C00,
                                                         buf2: 0x1A800,
                                                         size: 0x1400,
                                                     },
                                                     // `0x17C00`–`0x17FFF` (Something important)
                                                     SRAMSwapBuffers {
                                                         buf1: 0x17C00,
                                                         buf2: 0x1BC00,
                                                         size: 0x0400,
                                                     }];

/// *AR100* *SRAM* swap buffers.
///
/// Use the *SRAM* section at `0x44000` as the backup storage. This is the memory, which is
/// normally shared with the *OpenRISC* core (should we do an extra check to ensure that this core
/// is powered off and can't interfere?).
const AR100_ABUSING_SRAM_SWAP_BUFFERS: [SRAMSwapBuffers; 2] = [SRAMSwapBuffers {
                                                                   buf1: 0x1800,
                                                                   buf2: 0x44000,
                                                                   size: 0x800,
                                                               },
                                                               SRAMSwapBuffers {
                                                                   buf1: 0x5C00,
                                                                   buf2: 0x44800,
                                                                   size: 0x8000 - 0x5C00,
                                                               }];

/// *A80* *SRAM* swap buffers.
///
/// *A80* has *40KiB* *SRAM* *A1* at `0x10000` where the *SPL* has to be loaded to. The secure
/// *SRAM* *B* at `0x20000` is used as backup area for *FEL* stacks and data.
const A80_SRAM_SWAP_BUFFERS: [SRAMSwapBuffers; 2] = [SRAMSwapBuffers {
                                                         buf1: 0x11800,
                                                         buf2: 0x20000,
                                                         size: 0x800,
                                                     },
                                                     SRAMSwapBuffers {
                                                         buf1: 0x15400,
                                                         buf2: 0x20800,
                                                         size: 0x18000 - 0x15400,
                                                     }];

/// Table with all the supported *SoCs*.
const SOC_INFO_TABLE: [SocInfo; 12] = [SocInfo {
                                           soc_id: 0x1623, // Allwinner A10
                                           name: "A10",
                                           spl_addr: 0x00000000,
                                           scratch_addr: 0x1000,
                                           thunk_addr: 0xA200,
                                           thunk_size: 0x200,
                                           needs_l2en: true,
                                           mmu_tt_addr: None,
                                           sid_addr: Some(0x01C23800),
                                           rvbar_reg: None,
                                           swap_buffers: &A10_A13_A20_SRAM_SWAP_BUFFERS,
                                       },
                                       SocInfo {
                                           soc_id: 0x1625, // Allwinner A10s, A13, R8
                                           name: "A10s/A13/R8",
                                           spl_addr: 0x00000000,
                                           scratch_addr: 0x1000,
                                           thunk_addr: 0xA200,
                                           thunk_size: 0x200,
                                           needs_l2en: true,
                                           mmu_tt_addr: None,
                                           sid_addr: Some(0x01C23800),
                                           rvbar_reg: None,
                                           swap_buffers: &A10_A13_A20_SRAM_SWAP_BUFFERS,
                                       },
                                       SocInfo {
                                           soc_id: 0x1651, // Allwinner A20
                                           name: "A20",
                                           spl_addr: 0x00000000,
                                           scratch_addr: 0x1000,
                                           thunk_addr: 0xA200,
                                           thunk_size: 0x200,
                                           needs_l2en: false,
                                           mmu_tt_addr: None,
                                           sid_addr: Some(0x01C23800),
                                           rvbar_reg: None,
                                           swap_buffers: &A10_A13_A20_SRAM_SWAP_BUFFERS,
                                       },
                                       SocInfo {
                                           soc_id: 0x1650, // Allwinner A23
                                           name: "A23",
                                           spl_addr: 0x00000000,
                                           scratch_addr: 0x1000,
                                           thunk_addr: 0x46E00,
                                           thunk_size: 0x200,
                                           needs_l2en: false,
                                           mmu_tt_addr: None,
                                           sid_addr: Some(0x01C23800),
                                           rvbar_reg: None,
                                           swap_buffers: &AR100_ABUSING_SRAM_SWAP_BUFFERS,
                                       },
                                       SocInfo {
                                           soc_id: 0x1633, // Allwinner A31
                                           name: "A31",
                                           spl_addr: 0x00000000,
                                           scratch_addr: 0x1000,
                                           thunk_addr: 0x22E00,
                                           thunk_size: 0x200,
                                           needs_l2en: false,
                                           mmu_tt_addr: None,
                                           sid_addr: None,
                                           rvbar_reg: None,
                                           swap_buffers: &A31_SRAM_SWAP_BUFFERS,
                                       },
                                       SocInfo {
                                           soc_id: 0x1667, // Allwinner A33, R16
                                           name: "A33/R16",
                                           spl_addr: 0x00000000,
                                           scratch_addr: 0x1000,
                                           thunk_addr: 0x46E00,
                                           thunk_size: 0x200,
                                           needs_l2en: false,
                                           mmu_tt_addr: None,
                                           sid_addr: Some(0x01C23800),
                                           rvbar_reg: None,
                                           swap_buffers: &AR100_ABUSING_SRAM_SWAP_BUFFERS,
                                       },
                                       SocInfo {
                                           soc_id: 0x1689, // Allwinner A64
                                           name: "A64",
                                           spl_addr: 0x10000,
                                           scratch_addr: 0x11000,
                                           thunk_addr: 0x1A200,
                                           thunk_size: 0x200,
                                           needs_l2en: false,
                                           mmu_tt_addr: None,
                                           sid_addr: Some(0x01C14200),
                                           rvbar_reg: Some(0x017000A0),
                                           swap_buffers: &A64_SRAM_SWAP_BUFFERS,
                                       },
                                       SocInfo {
                                           soc_id: 0x1639, // Allwinner A80
                                           name: "A80",
                                           spl_addr: 0x10000,
                                           scratch_addr: 0x11000,
                                           thunk_addr: 0x23400,
                                           thunk_size: 0x200,
                                           needs_l2en: false,
                                           mmu_tt_addr: None,
                                           sid_addr: Some(0x01C0E200),
                                           rvbar_reg: None,
                                           swap_buffers: &A80_SRAM_SWAP_BUFFERS,
                                       },
                                       SocInfo {
                                           soc_id: 0x1673, // Allwinner A83T
                                           name: "A83T",
                                           spl_addr: 0x00000000,
                                           scratch_addr: 0x1000,
                                           thunk_addr: 0x46E00,
                                           thunk_size: 0x200,
                                           needs_l2en: false,
                                           mmu_tt_addr: None,
                                           sid_addr: Some(0x01C14200),
                                           rvbar_reg: None,
                                           swap_buffers: &AR100_ABUSING_SRAM_SWAP_BUFFERS,
                                       },
                                       SocInfo {
                                           soc_id: 0x1680, // Allwinner H3, H2+
                                           name: "H3/H2+",
                                           spl_addr: 0x00000000,
                                           scratch_addr: 0x1000,
                                           thunk_addr: 0xA200,
                                           thunk_size: 0x200,
                                           needs_l2en: false,
                                           mmu_tt_addr: Some(0x8000),
                                           sid_addr: Some(0x01C14200),
                                           rvbar_reg: None,
                                           swap_buffers: &A10_A13_A20_SRAM_SWAP_BUFFERS,
                                       },
                                       SocInfo {
                                           soc_id: 0x1718, // Allwinner H5
                                           name: "H5",
                                           spl_addr: 0x10000,
                                           scratch_addr: 0x11000,
                                           thunk_addr: 0x1A200,
                                           thunk_size: 0x200,
                                           needs_l2en: false,
                                           mmu_tt_addr: None,
                                           sid_addr: Some(0x01C14200),
                                           rvbar_reg: Some(0x017000A0),
                                           swap_buffers: &A64_SRAM_SWAP_BUFFERS,
                                       },
                                       SocInfo {
                                           soc_id: 0x1701, // Allwinner R40
                                           name: "R40",
                                           spl_addr: 0x00000000,
                                           scratch_addr: 0x1000,
                                           thunk_addr: 0xA200,
                                           thunk_size: 0x200,
                                           needs_l2en: false,
                                           mmu_tt_addr: None,
                                           sid_addr: Some(0x01C1B200),
                                           rvbar_reg: None,
                                           swap_buffers: &A10_A13_A20_SRAM_SWAP_BUFFERS,
                                       }];

/// *SoC* version information, as retrieved by the *FEL* protocol
#[derive(PartialEq)]
pub struct SocVersion {
    signature: [u8; 8],
    /// `0x00162300`.
    soc_id: u32,
    // `1`.
    // unknown_0a: u32,
    /// `1`.
    protocol: u16,
    // `0x44`.
    // unknown_12: u8,
    // `0x08`.
    // unknown_13: u8,
    /// `0x7e00`.
    scratchpad: u32, // unused: pad: [u32; 2],
}

impl SocVersion {
    /// Generates a SoC version information structure from the bytes response.
    #[doc(hidden)]
    pub fn from_bytes(bytes: [u8; 32]) -> SocVersion {
        use byteorder::{LittleEndian, ByteOrder};

        let mut signature = [0u8; 8];
        signature.clone_from_slice(&bytes[..8]);

        SocVersion {
            signature: signature,
            soc_id: (LittleEndian::read_u32(&bytes[8..12]) >> 8) & 0xFFFF,
            protocol: LittleEndian::read_u16(&bytes[16..18]),
            scratchpad: LittleEndian::read_u32(&bytes[20..24]),
        }
    }

    /// Gets the SoC ID.
    pub fn get_id(&self) -> u32 {
        self.soc_id
    }
}

impl fmt::Debug for SocVersion {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,
               "Signature: {}, SoC ID: {:#010x} ({}), protocol: {:#06x}, scratchpad: {:#010x}",
               String::from_utf8_lossy(&self.signature),
               self.soc_id,
               if let Some(name) = get_soc_name_from_id(self.soc_id) {
                   name
               } else {
                   "unknown"
               },
               self.protocol,
               self.scratchpad)
    }
}

/// Gets the *SoC* name from the given ID, if supported.
fn get_soc_name_from_id(soc_id: u32) -> Option<&'static str> {
    for soc_info in &SOC_INFO_TABLE {
        if soc_info.soc_id == soc_id {
            return Some(soc_info.name);
        }
    }
    None
}

/// *SRAM* buffers.
///
/// The `SRAMSwapBuffers` structure is used to describe information about pairwise memory regions
/// in *SRAM*, the content of which needs to be exchanged before calling the U-Boot *SPL* code and
/// then exchanged again before returning control back to the *FEL* code from the *BROM*.
#[derive(PartialEq)]
pub struct SRAMSwapBuffers {
    /// BROM buffer.
    buf1: u32,
    /// Backup storage location.
    buf2: u32,
    /// Buffer size.
    size: u32,
}

impl SRAMSwapBuffers {
    /// Gets the BROM buffer.
    pub fn get_buf1(&self) -> u32 {
        self.buf1
    }

    /// Gets the backup storage location.
    pub fn get_buf2(&self) -> u32 {
        self.buf2
    }

    /// Gets the buffer size.
    pub fn get_size(&self) -> u32 {
        self.size
    }
}

impl fmt::Debug for SRAMSwapBuffers {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,
               "{{ BROM buffer: {:#010x}, backup storage: {:#010x}, buffer size: {:#010x} }}",
               self.buf1,
               self.buf2,
               self.size)
    }
}

/// *SoC* information structure.
///
/// Each *SoC* variant may have its own list of memory buffers to be exchanged and the information
/// about the placement of the thunk code, which handles the transition of execution from the
/// *BROM* *FEL* code to the U-Boot *SPL* and back.
///
/// **Note:** the entries in the `swap_buffers` tables need to be sorted by `buf1` addresses. And
/// the `buf1` addresses are the *BROM* data buffers, while `buf2` addresses are the intended
/// backup locations.
///
/// Also for performance reasons, we optionally want to have *MMU* enabled with optimal section
/// attributes configured (the code from the *BROM* should use *I-cache*, writing data to the
/// *DRAM* area should use write combining). The reason is that the *BROM* *FEL* protocol
/// implementation moves data using the *CPU* somewhere on the performance critical path when
/// transferring data over *USB*. The older *SoC* variants (*A10*/*A13*/*A20*/*A31*/*A23*) already
/// have *MMU* enabled and we only need to adjust section attributes. The *BROM* in newer *SoC*
/// variants (*A33*/*A83T*/*H3*) doesn't enable *MMU* any more, so we need to find some 16K of
/// spare space in *SRAM* to place the translation table there and specify it as the `mmu_tt_addr`
/// field in the `soc_sram_info` structure. The `mmu_tt_addr` address must be 16K aligned.
#[derive(PartialEq, Clone, Copy)]
pub struct SocInfo {
    /// ID of the SoC.
    soc_id: u32,
    /// Human-readable SoC name string.
    name: &'static str,
    /// SPL load address.
    spl_addr: u32,
    /// A safe place to upload & run code.
    scratch_addr: u32,
    /// Address of the thunk code.
    thunk_addr: u32,
    /// Maximal size of the thunk code.
    thunk_size: u32,
    /// Set the `L2EN` bit.
    needs_l2en: bool,
    /// MMU translation table address.
    mmu_tt_addr: Option<u32>,
    /// Base address for `SID_KEY[0-3]` registers.
    sid_addr: Option<u32>,
    /// MMIO address of `RVBARADDR0_L` register.
    rvbar_reg: Option<u32>,
    /// SRAM buffers.
    swap_buffers: &'static [SRAMSwapBuffers],
}

impl SocInfo {
    /// Gets the SoC information structure for the given ID, if supported.
    pub fn from_id(soc_id: u32) -> Option<SocInfo> {
        for soc_info in &SOC_INFO_TABLE {
            if soc_info.soc_id == soc_id {
                return Some(*soc_info);
            }
        }
        None
    }

    /// Gets the SoC information structure from the givern `SoCVersion`, if supported.
    #[doc(hidden)]
    pub fn from_version(version: &SocVersion) -> Option<SocInfo> {
        SocInfo::from_id(version.soc_id)
    }

    /// Gets the SoC ID.
    pub fn get_soc_id(&self) -> u32 {
        self.soc_id
    }

    /// Gets the name of the SoC.
    pub fn get_name(&self) -> &str {
        self.name
    }

    /// Gets the SPL load address for the SoC.
    #[doc(hidden)]
    pub fn get_spl_addr(&self) -> u32 {
        self.spl_addr
    }

    /// Gets the scratch address.
    #[doc(hidden)]
    pub fn get_scratch_addr(&self) -> u32 {
        self.scratch_addr
    }

    /// Gets the address of the thunk code.
    #[doc(hidden)]
    pub fn get_thunk_addr(&self) -> u32 {
        self.thunk_addr
    }

    /// Gets the size of the thunk code.
    #[doc(hidden)]
    pub fn get_thunk_size(&self) -> u32 {
        self.thunk_size
    }

    // Does the SoC need L2 cache?
    #[doc(hidden)]
    pub fn needs_l2en(&self) -> bool {
        self.needs_l2en
    }

    /// Gets the *MMU* translation table address.
    #[doc(hidden)]
    pub fn get_mmu_tt_addr(&self) -> Option<u32> {
        self.mmu_tt_addr
    }

    /// Gets the `SID` register address.
    #[doc(hidden)]
    pub fn get_sid_addr(&self) -> Option<u32> {
        self.sid_addr
    }

    /// Get the `RVBAR` register address, if supported.
    #[doc(hidden)]
    pub fn get_rvbar_reg(&self) -> Option<u32> {
        self.rvbar_reg
    }

    /// Gets the SoC swap buffers.
    #[doc(hidden)]
    pub fn get_swap_buffers(&self) -> &[SRAMSwapBuffers] {
        self.swap_buffers
    }
}

impl fmt::Debug for SocInfo {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,
               "{{ SoC ID: {:#010x}, name: {}, SPL load address: {:#010x}, scratch address: \
                {:#010x}, thunk address: {:#010x}, thunk size: {:#010x}, L2EN bit: {}, MMU \
                translation table address: {}, SID registers address: {}, MMIO address of \
                `RVBARADDR0_L` register: {}, SRAM swap buffers: {:?} }}",
               self.soc_id,
               self.name,
               self.spl_addr,
               self.scratch_addr,
               self.thunk_addr,
               self.thunk_size,
               self.needs_l2en,
               if let Some(mmu_tt_addr) = self.mmu_tt_addr {
                   format!("{:#010x}", mmu_tt_addr)
               } else {
                   "None".to_owned()
               },
               if let Some(sid_addr) = self.sid_addr {
                   format!("{:#010x}", sid_addr)
               } else {
                   "None".to_owned()
               },
               if let Some(rvbar_reg) = self.rvbar_reg {
                   format!("{:#010x}", rvbar_reg)
               } else {
                   "None".to_owned()
               },
               self.swap_buffers)
    }
}

#[cfg(test)]
mod tests {
    use super::{SOC_INFO_TABLE, A10_A13_A20_SRAM_SWAP_BUFFERS, A31_SRAM_SWAP_BUFFERS,
                A64_SRAM_SWAP_BUFFERS, AR100_ABUSING_SRAM_SWAP_BUFFERS, A80_SRAM_SWAP_BUFFERS};
    use super::SRAMSwapBuffers;

    #[test]
    fn it_a10_a13_a20_sram_swap_buffers() {
        let buffers = A10_A13_A20_SRAM_SWAP_BUFFERS;

        assert_eq!(buffers[0],
                   SRAMSwapBuffers {
                       buf1: 0x1C00,
                       buf2: 0xA400,
                       size: 0x0400,
                   });
        assert_eq!(buffers[1],
                   SRAMSwapBuffers {
                       buf1: 0x5C00,
                       buf2: 0xA800,
                       size: 0x1400,
                   });
        assert_eq!(buffers[2],
                   SRAMSwapBuffers {
                       buf1: 0x7C00,
                       buf2: 0xBC00,
                       size: 0x0400,
                   });
    }

    #[test]
    fn it_a31_sram_swap_buffers() {
        let buffers = A31_SRAM_SWAP_BUFFERS;

        assert_eq!(buffers[0],
                   SRAMSwapBuffers {
                       buf1: 0x1800,
                       buf2: 0x20000,
                       size: 0x800,
                   });
        assert_eq!(buffers[1],
                   SRAMSwapBuffers {
                       buf1: 0x5C00,
                       buf2: 0x20800,
                       size: 0x8000 - 0x5C00,
                   });
    }

    #[test]
    fn it_a64_sram_swap_buffers() {
        let buffers = A64_SRAM_SWAP_BUFFERS;

        assert_eq!(buffers[0],
                   SRAMSwapBuffers {
                       buf1: 0x11C00,
                       buf2: 0x1A400,
                       size: 0x0400,
                   });
        assert_eq!(buffers[1],
                   SRAMSwapBuffers {
                       buf1: 0x15C00,
                       buf2: 0x1A800,
                       size: 0x1400,
                   });
        assert_eq!(buffers[2],
                   SRAMSwapBuffers {
                       buf1: 0x17C00,
                       buf2: 0x1BC00,
                       size: 0x0400,
                   });
    }

    #[test]
    fn it_ar100_abusing_sram_swap_buffers() {
        let buffers = AR100_ABUSING_SRAM_SWAP_BUFFERS;

        assert_eq!(buffers[0],
                   SRAMSwapBuffers {
                       buf1: 0x1800,
                       buf2: 0x44000,
                       size: 0x800,
                   });
        assert_eq!(buffers[1],
                   SRAMSwapBuffers {
                       buf1: 0x5C00,
                       buf2: 0x44800,
                       size: 0x8000 - 0x5C00,
                   });
    }

    #[test]
    fn it_a80_sram_swap_buffers() {
        let buffers = A80_SRAM_SWAP_BUFFERS;

        assert_eq!(buffers[0],
                   SRAMSwapBuffers {
                       buf1: 0x11800,
                       buf2: 0x20000,
                       size: 0x800,
                   });
        assert_eq!(buffers[1],
                   SRAMSwapBuffers {
                       buf1: 0x15400,
                       buf2: 0x20800,
                       size: 0x18000 - 0x15400,
                   });
    }

    #[test]
    fn it_soc_info() {
        for soc_info in &SOC_INFO_TABLE {
            match soc_info.soc_id {
                0x1623 => {
                    // Allwinner A10
                    assert_eq!(soc_info.name, "A10");
                    assert_eq!(soc_info.spl_addr, 0x00000000);
                    assert_eq!(soc_info.scratch_addr, 0x1000);
                    assert!(soc_info.mmu_tt_addr.is_none());
                    assert!(soc_info.mmu_tt_addr.is_none());
                    assert_eq!(soc_info.thunk_addr, 0xA200);
                    assert_eq!(soc_info.thunk_size, 0x200);
                    assert_eq!(soc_info.swap_buffers, &A10_A13_A20_SRAM_SWAP_BUFFERS);
                    assert!(soc_info.needs_l2en);
                    assert_eq!(soc_info.sid_addr, Some(0x01C23800));
                    assert!(soc_info.rvbar_reg.is_none());
                }
                0x1625 => {
                    // Allwinner A10s, A13, R8
                    assert_eq!(soc_info.name, "A10s/A13/R8");
                    assert_eq!(soc_info.spl_addr, 0x00000000);
                    assert_eq!(soc_info.scratch_addr, 0x1000);
                    assert!(soc_info.mmu_tt_addr.is_none());
                    assert_eq!(soc_info.thunk_addr, 0xA200);
                    assert_eq!(soc_info.thunk_size, 0x200);
                    assert_eq!(soc_info.swap_buffers, &A10_A13_A20_SRAM_SWAP_BUFFERS);
                    assert!(soc_info.needs_l2en);
                    assert_eq!(soc_info.sid_addr, Some(0x01C23800));
                    assert!(soc_info.rvbar_reg.is_none());
                }
                0x1651 => {
                    // Allwinner A20
                    assert_eq!(soc_info.name, "A20");
                    assert_eq!(soc_info.spl_addr, 0x00000000);
                    assert_eq!(soc_info.scratch_addr, 0x1000);
                    assert!(soc_info.mmu_tt_addr.is_none());
                    assert_eq!(soc_info.thunk_addr, 0xA200);
                    assert_eq!(soc_info.thunk_size, 0x200);
                    assert_eq!(soc_info.swap_buffers, &A10_A13_A20_SRAM_SWAP_BUFFERS);
                    assert!(!soc_info.needs_l2en);
                    assert_eq!(soc_info.sid_addr, Some(0x01C23800));
                    assert!(soc_info.rvbar_reg.is_none());
                }
                0x1650 => {
                    // Allwinner A23
                    assert_eq!(soc_info.name, "A23");
                    assert_eq!(soc_info.spl_addr, 0x00000000);
                    assert_eq!(soc_info.scratch_addr, 0x1000);
                    assert!(soc_info.mmu_tt_addr.is_none());
                    assert_eq!(soc_info.thunk_addr, 0x46E00);
                    assert_eq!(soc_info.thunk_size, 0x200);
                    assert_eq!(soc_info.swap_buffers, &AR100_ABUSING_SRAM_SWAP_BUFFERS);
                    assert!(!soc_info.needs_l2en);
                    assert_eq!(soc_info.sid_addr, Some(0x01C23800));
                    assert!(soc_info.rvbar_reg.is_none());
                }
                0x1633 => {
                    // Allwinner A31
                    assert_eq!(soc_info.name, "A31");
                    assert_eq!(soc_info.spl_addr, 0x00000000);
                    assert_eq!(soc_info.scratch_addr, 0x1000);
                    assert!(soc_info.mmu_tt_addr.is_none());
                    assert_eq!(soc_info.thunk_addr, 0x22E00);
                    assert_eq!(soc_info.thunk_size, 0x200);
                    assert_eq!(soc_info.swap_buffers, &A31_SRAM_SWAP_BUFFERS);
                    assert!(!soc_info.needs_l2en);
                    assert!(soc_info.sid_addr.is_none());
                    assert!(soc_info.rvbar_reg.is_none());
                }
                0x1667 => {
                    // Allwinner A33, R16
                    assert_eq!(soc_info.name, "A33/R16");
                    assert_eq!(soc_info.spl_addr, 0x00000000);
                    assert_eq!(soc_info.scratch_addr, 0x1000);
                    assert!(soc_info.mmu_tt_addr.is_none());
                    assert_eq!(soc_info.thunk_addr, 0x46E00);
                    assert_eq!(soc_info.thunk_size, 0x200);
                    assert_eq!(soc_info.swap_buffers, &AR100_ABUSING_SRAM_SWAP_BUFFERS);
                    assert!(!soc_info.needs_l2en);
                    assert_eq!(soc_info.sid_addr, Some(0x01C23800));
                    assert!(soc_info.rvbar_reg.is_none());
                }
                0x1689 => {
                    // Allwinner A64
                    assert_eq!(soc_info.name, "A64");
                    assert_eq!(soc_info.spl_addr, 0x10000);
                    assert_eq!(soc_info.scratch_addr, 0x11000);
                    assert!(soc_info.mmu_tt_addr.is_none());
                    assert_eq!(soc_info.thunk_addr, 0x1A200);
                    assert_eq!(soc_info.thunk_size, 0x200);
                    assert_eq!(soc_info.swap_buffers, &A64_SRAM_SWAP_BUFFERS);
                    assert!(!soc_info.needs_l2en);
                    assert_eq!(soc_info.sid_addr, Some(0x01C14200));
                    assert_eq!(soc_info.rvbar_reg, Some(0x017000A0));
                }
                0x1639 => {
                    // Allwinner A80
                    assert_eq!(soc_info.name, "A80");
                    assert_eq!(soc_info.spl_addr, 0x10000);
                    assert_eq!(soc_info.scratch_addr, 0x11000);
                    assert!(soc_info.mmu_tt_addr.is_none());
                    assert_eq!(soc_info.thunk_addr, 0x23400);
                    assert_eq!(soc_info.thunk_size, 0x200);
                    assert_eq!(soc_info.swap_buffers, &A80_SRAM_SWAP_BUFFERS);
                    assert!(!soc_info.needs_l2en);
                    assert_eq!(soc_info.sid_addr, Some(0x01c0e200));
                    assert!(soc_info.rvbar_reg.is_none());
                }
                0x1673 => {
                    // Allwinner A83T
                    assert_eq!(soc_info.name, "A83T");
                    assert_eq!(soc_info.spl_addr, 0x00000000);
                    assert_eq!(soc_info.scratch_addr, 0x1000);
                    assert!(soc_info.mmu_tt_addr.is_none());
                    assert_eq!(soc_info.thunk_addr, 0x46E00);
                    assert_eq!(soc_info.thunk_size, 0x200);
                    assert_eq!(soc_info.swap_buffers, &AR100_ABUSING_SRAM_SWAP_BUFFERS);
                    assert!(!soc_info.needs_l2en);
                    assert_eq!(soc_info.sid_addr, Some(0x01C14200));
                    assert!(soc_info.rvbar_reg.is_none());
                }
                0x1680 => {
                    // Allwinner H3, H2+
                    assert_eq!(soc_info.name, "H3/H2+");
                    assert_eq!(soc_info.spl_addr, 0x00000000);
                    assert_eq!(soc_info.scratch_addr, 0x1000);
                    assert_eq!(soc_info.mmu_tt_addr, Some(0x8000));
                    assert_eq!(soc_info.thunk_addr, 0xA200);
                    assert_eq!(soc_info.thunk_size, 0x200);
                    assert_eq!(soc_info.swap_buffers, &A10_A13_A20_SRAM_SWAP_BUFFERS);
                    assert!(!soc_info.needs_l2en);
                    assert_eq!(soc_info.sid_addr, Some(0x01C14200));
                    assert!(soc_info.rvbar_reg.is_none());
                }
                0x1718 => {
                    // Allwinner H5
                    assert_eq!(soc_info.name, "H5");
                    assert_eq!(soc_info.spl_addr, 0x10000);
                    assert_eq!(soc_info.scratch_addr, 0x11000);
                    assert!(soc_info.mmu_tt_addr.is_none());
                    assert_eq!(soc_info.thunk_addr, 0x1A200);
                    assert_eq!(soc_info.thunk_size, 0x200);
                    assert_eq!(soc_info.swap_buffers, &A64_SRAM_SWAP_BUFFERS);
                    assert!(!soc_info.needs_l2en);
                    assert_eq!(soc_info.sid_addr, Some(0x01C14200));
                    assert_eq!(soc_info.rvbar_reg, Some(0x017000A0));
                }
                0x1701 => {
                    // Allwinner R40
                    assert_eq!(soc_info.name, "R40");
                    assert_eq!(soc_info.spl_addr, 0x00000000);
                    assert_eq!(soc_info.scratch_addr, 0x1000);
                    assert!(soc_info.mmu_tt_addr.is_none());
                    assert_eq!(soc_info.thunk_addr, 0xA200);
                    assert_eq!(soc_info.thunk_size, 0x200);
                    assert_eq!(soc_info.swap_buffers, &A10_A13_A20_SRAM_SWAP_BUFFERS);
                    assert!(!soc_info.needs_l2en);
                    assert_eq!(soc_info.sid_addr, Some(0x01C1B200));
                    assert!(soc_info.rvbar_reg.is_none());
                }
                _ => unreachable!(),
            }
        }
    }
}
