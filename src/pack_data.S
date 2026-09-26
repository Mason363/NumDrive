/* the game data pack produced by tools/pack.py */
  .section .rodata.pack_data, "a"
  .balign 4
  .global pack_data
pack_data:
  .incbin "pack.bin"
pack_end:
  .balign 4
  .global pack_size
pack_size:
  .word pack_end - pack_data
#if defined(__linux__) && defined(__ELF__)
  .section .note.GNU-stack, "", %progbits
#endif
