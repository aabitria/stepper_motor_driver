	.text
	.syntax unified
	.thumb

	.type root2_s, %function
	.global root2_s
root2_s:
	VSQRT.F32 S0, S0
   	BX LR

   	.type root2_s, %function
	.global root2_s
root2n_s:
 	CMP R0, #2
 	BCC _end

	// argument is greater or equal to 2, so (hopefully) 2, 4, .. 2^n
 	// test and loop
 	// Test is still valid
_root2n_s$$1:
 	CMP R0, #2
 	ITTT GE
 	VSQRTGE.F32 S0, S0
 	MOVGE R0, R0, ASR #1
 	BGE _root2n_s$$1
_end:
 	BX LR

	.end
