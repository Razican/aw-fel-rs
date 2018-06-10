pub const FEL_TO_SPL_THUNK: [u32; 66] = [
    //   0:    b          5c <setup_stack>
    0x_ea_00_00_15,
    //   4:    nop
    0x_e1_a0_00_00,
    //   8:    nop
    0x_e1_a0_00_00,
    //   c:    nop
    0x_e1_a0_00_00,
    //  10:    nop
    0x_e1_a0_00_00,
    //  14:    nop
    0x_e1_a0_00_00,
    //  18:    nop
    0x_e1_a0_00_00,
    //  1c:    nop
    0x_e1_a0_00_00,
    //  20:    nop
    0x_e1_a0_00_00,
    //  24:    nop
    0x_e1_a0_00_00,
    //  28:    add        r4, pc, #220
    0x_e2_8f_40_dc,
    //  2c:    ldr        r0, [r4], #4
    0x_e4_94_00_04,
    //  30:    ldr        r1, [r4], #4
    0x_e4_94_10_04,
    //  34:    ldr        r6, [r4], #4
    0x_e4_94_60_04,
    //  38:    cmp        r6, #0
    0x_e3_56_00_00,
    //  3c:    bxeq       lr
    0x_01_2f_ff_1e,
    //  40:    ldr        r2, [r0]
    0x_e5_90_20_00,
    //  44:    ldr        r3, [r1]
    0x_e5_91_30_00,
    //  48:    subs       r6, r6, #4
    0x_e2_56_60_04,
    //  4c:    str        r2, [r1], #4
    0x_e4_81_20_04,
    //  50:    str        r3, [r0], #4
    0x_e4_80_30_04,
    //  54:    bne        40 <swap_next_word>
    0x_1a_ff_ff_f9,
    //  58:    b          2c <swap_next_buffer>
    0x_ea_ff_ff_f3,
    //  5c:    ldr        r8, [pc, #164]
    0x_e5_9f_80_a4,
    //  60:    sub        r0, pc, #68
    0x_e2_4f_00_44,
    //  64:    str        sp, [r0, #-4]!
    0x_e5_20_d0_04,
    //  68:    mov        sp, r0
    0x_e1_a0_d0_00,
    //  6c:    mrs        r2, CPSR
    0x_e1_0f_20_00,
    //  70:    push       {r2, lr}
    0x_e9_2d_40_04,
    //  74:    orr        r2, r2, #192
    0x_e3_82_20_c0,
    //  78:    msr        CPSR_c, r2
    0x_e1_21_f0_02,
    //  7c:    mrc        15, 0, r2, cr1, cr0, {0}
    0x_ee_11_2f_10,
    //  80:    movw       r3, #4100
    0x_e3_01_30_04,
    //  84:    tst        r2, r3
    0x_e1_12_00_03,
    //  88:    bne        d8 <cache_is_unsupported>
    0x_1a_00_00_12,
    //  8c:    bl         28 <swap_all_buffers>
    0x_eb_ff_ff_e5,
    //  90:    movw       r7, #27705
    0x_e3_06_7c_39,
    //  94:    movt       r7, #24330
    0x_e3_45_7f_0a,
    //  98:    mov        r0, r8
    0x_e1_a0_00_08,
    //  9c:    ldr        r5, [r0, #16]
    0x_e5_90_50_10,
    //  a0:    ldr        r2, [r0], #4
    0x_e4_90_20_04,
    //  a4:    subs       r5, r5, #4
    0x_e2_55_50_04,
    //  a8:    add        r7, r7, r2
    0x_e0_87_70_02,
    //  ac:    bne        a0 <check_next_word>
    0x_1a_ff_ff_fb,
    //  b0:    ldr        r2, [r8, #12]
    0x_e5_98_20_0c,
    //  b4:    subs       r7, r7, r2, lsl #1
    0x_e0_57_70_82,
    //  b8:    bne        e8 <checksum_is_bad>
    0x_1a_00_00_0a,
    //  bc:    movw       r2, #17966
    0x_e3_04_26_2e,
    //  c0:    movt       r2, #19525
    0x_e3_44_2c_45,
    //  c4:    str        r2, [r8, #8]
    0x_e5_88_20_08,
    //  c8:    dsb        sy
    0x_f5_7f_f0_4f,
    //  cc:    isb        sy
    0x_f5_7f_f0_6f,
    //  d0:    blx        r8
    0x_e1_2f_ff_38,
    //  d4:    b          f4 <return_to_fel>
    0x_ea_00_00_06,
    //  d8:    movw       r2, #16174
    0x_e3_03_2f_2e,
    //  dc:    movt       r2, #16191
    0x_e3_43_2f_3f,
    //  e0:    str        r2, [r8, #8]
    0x_e5_88_20_08,
    //  e4:    b          f8 <return_to_fel_noswap>
    0x_ea_00_00_03,
    //  e8:    movw       r2, #16942
    0x_e3_04_22_2e,
    //  ec:    movt       r2, #17473
    0x_e3_44_24_41,
    //  f0:    str        r2, [r8, #8]
    0x_e5_88_20_08,
    //  f4:    bl         28 <swap_all_buffers>
    0x_eb_ff_ff_cb,
    //  f8:    pop        {r2, lr}
    0x_e8_bd_40_04,
    //  fc:    msr        CPSR_c, r2
    0x_e1_21_f0_02,
    // 100:    ldr        sp, [sp]
    0x_e5_9d_d0_00,
    // 104:    bx         lr
    0x_e1_2f_ff_1e,
];
