		.section .piggydata,#alloc
		.globl input_data
input_data:
		.incbin        "arch/arm/boot/compressed/piggy.lz4.S"
		.globl input_data_end
input_data_end:

